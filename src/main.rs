use anyhow::{Context, Result, bail};
use evdev_rs::{
	Device, DeviceWrapper, GrabMode, InputEvent, ReadFlag, ReadStatus, UInputDevice,
	enums::{EV_ABS, EventCode, EventType},
};
use std::{
	collections::HashMap,
	io::Read,
	path::{Path, PathBuf},
	sync::{
		Arc,
		atomic::{AtomicBool, Ordering},
	},
	thread,
	time::Duration,
};

#[derive(Debug, Clone)]
pub struct DeviceInfo {
	pub name: String,
	pub path: PathBuf,
	pub phys: String,
	pub vendor_id: u16,
	pub product_id: u16,
	pub version: u16,
}

impl DeviceInfo {
	pub fn with_path(path: PathBuf) -> Result<Self> {
		let f = std::fs::File::open(&path).context(format!("opening {}", path.display()))?;
		let input = Device::new_from_file(f)
			.with_context(|| format!("failed to create new Device from file {}", path.display()))?;

		Ok(Self {
			name: input.name().unwrap_or("").to_string(),
			phys: input.phys().unwrap_or("").to_string(),
			vendor_id: input.vendor_id() as u16,
			product_id: input.product_id() as u16,
			version: input.version() as u16,
			path,
		})
	}

	pub fn with_name(
		name: &str,
		phys: Option<&str>,
		device_ids: Option<(u16, u16, u16)>,
	) -> Result<Self> {
		let mut devices = Self::obtain_device_list()?;

		// First try to match by VID/PID/Version if provided
		if let Some((vid, pid, version)) = device_ids {
			match devices.iter().position(|item| {
				item.vendor_id == vid && item.product_id == pid && item.version == version
			}) {
				Some(idx) => return Ok(devices.remove(idx)),
				None => {
					bail!(
						"Requested device with VID={:04x}/PID={:04x}/Version={:04x} was not found",
						vid,
						pid,
						version
					);
				}
			}
		}

		// Then try to match by phys if provided
		if let Some(phys) = phys {
			match devices.iter().position(|item| item.phys == phys) {
				Some(idx) => return Ok(devices.remove(idx)),
				None => {
					bail!(
						"Requested device `{}` with phys=`{}` was not found",
						name,
						phys
					);
				}
			}
		}

		let mut devices_with_name: Vec<_> = devices
			.into_iter()
			.filter(|item| item.name == name)
			.collect();

		if devices_with_name.is_empty() {
			bail!("No device found with name `{}`", name);
		}

		if devices_with_name.len() > 1 {
			eprintln!("Multiple devices match name `{}`, using first entry:", name);
			for dev in &devices_with_name {
				eprintln!(
					"{:?} (VID={:04x}/PID={:04x}/Version={:04x})",
					dev, dev.vendor_id, dev.product_id, dev.version
				);
			}
		}

		Ok(devices_with_name.remove(0))
	}

	fn obtain_device_list() -> Result<Vec<DeviceInfo>> {
		let mut devices = vec![];
		for entry in std::fs::read_dir("/dev/input")? {
			let entry = entry?;

			if !entry
				.file_name()
				.to_str()
				.unwrap_or("")
				.starts_with("event")
			{
				continue;
			}
			let path = entry.path();
			if path.is_dir() {
				continue;
			}

			match DeviceInfo::with_path(path) {
				Ok(item) => devices.push(item),
				Err(err) => eprintln!("{:#}", err),
			}
		}

		// Order by name, but when multiple devices have the same name,
		// order by the event device unit number
		devices.sort_by(|a, b| {
			match a.name.cmp(&b.name) {
				std::cmp::Ordering::Equal => {
					// Extract event numbers for comparison
					let a_num = a
						.path
						.file_name()
						.and_then(|n| n.to_str())
						.and_then(|s| s.strip_prefix("event"))
						.and_then(|n| n.parse::<u32>().ok())
						.unwrap_or(u32::MAX);
					let b_num = b
						.path
						.file_name()
						.and_then(|n| n.to_str())
						.and_then(|s| s.strip_prefix("event"))
						.and_then(|n| n.parse::<u32>().ok())
						.unwrap_or(u32::MAX);
					a_num.cmp(&b_num)
				}
				other => other,
			}
		});

		Ok(devices)
	}
}

/// Represents the state of the remapper
#[derive(Debug, Clone, PartialEq)]
pub enum RemapperState {
	Starting,
	Running,
	Error(String),
	Stopped,
	NoGrab,
}

/// NURBS curve configuration
#[derive(Clone, Debug)]
struct CurveConfig {
	/// Control points for the curve
	pub control_points: Vec<Vec<f64>>,
	/// Knot vector for the curve
	pub knots: Vec<f64>,
	/// Weights for the control points
	pub weights: Vec<f64>,
	/// Degree of the curve
	pub degree: usize,
}

impl CurveConfig {
	pub fn eval(&self, t: f64) -> Result<Vec<f64>, &'static str> {
		todo!()
	}
}

/// Configuration for a single axis remapping
#[derive(Clone, Debug)]
pub struct AxisConfig {
	nurbs_curve: Option<CurveConfig>,
}

pub struct InputMapper {
	input: Device,
	output: UInputDevice,
}

impl InputMapper {
	pub fn create_mapper<P: AsRef<Path>>(path: P) -> Result<Self> {
		let path = path.as_ref();
		let f = std::fs::File::open(path).context(format!("opening {}", path.display()))?;
		let mut input = Device::new_from_file(f)
			.with_context(|| format!("failed to create new Device from file {}", path.display()))?;

		input.set_name(&format!("evremap Virtual input for {}", path.display()));

		let output = UInputDevice::create_from_device(&input)
			.context(format!("creating UInputDevice from {}", path.display()))?;

		input
			.grab(GrabMode::Grab)
			.context(format!("grabbing exclusive access on {}", path.display()))?;

		Ok(Self { input, output })
	}
}

/// Main remapper struct
pub struct FlightstickCurveApp {
	source_path: PathBuf,
	state: RemapperState,
	running: Arc<AtomicBool>,
	axis_configs: HashMap<u16, AxisConfig>,
}

impl FlightstickCurveApp {
	pub fn new(source_path: PathBuf) -> Self {
		Self {
			source_path,
			state: RemapperState::Starting,
			running: Arc::new(AtomicBool::new(false)),
			axis_configs: HashMap::new(),
		}
	}

	/// Configure axis remapping
	pub fn configure_axis(&mut self, axis_code: u16, config: AxisConfig) {
		self.axis_configs.insert(axis_code, config);
	}

	/// Start the remapper
	pub fn start(&mut self) -> Result<()> {
		println!(
			"Starting flightstick-nurbs for device: {}",
			&self.source_path.display()
		);

		// Create virtual device with copied capabilities
		let mut virtual_device = InputMapper::create_mapper(&self.source_path)?;

		// Set running flag
		self.running.store(true, Ordering::SeqCst);

		// Start event processing thread
		self.state = RemapperState::Running;
		let running = Arc::clone(&self.running);

		// Clone axis configs for the thread
		let axis_configs = self.axis_configs.clone();

		thread::spawn(move || {
			// Create a remapper for the thread
			let thread_remapper = ThreadRemapper { axis_configs };

			while running.load(Ordering::SeqCst) {
				// Read events with timeout
				if let Ok((status, event)) = virtual_device
					.input
					.next_event(ReadFlag::NORMAL | ReadFlag::BLOCKING)
				{
					match status {
						ReadStatus::Success => {
							if let Some(modified_event) = thread_remapper.process_event(event) {
								eprintln!("Modified event: {:?}", modified_event);
								if let Err(e) = virtual_device.output.write_event(&modified_event) {
									eprintln!("Error writing event: {}", e);
								}
							}
						}
						ReadStatus::Sync => todo!(),
					}
				}
				thread::sleep(Duration::from_micros(100));
			}
			virtual_device.input.grab(GrabMode::Ungrab).unwrap();
		});

		Ok(())
	}

	/// Stop the remapper
	pub fn stop(&mut self) -> Result<()> {
		println!("Stopping input remapper");

		// Set the running flag to false to stop the thread
		self.running.store(false, Ordering::SeqCst);

		// Give the thread a moment to exit
		thread::sleep(Duration::from_millis(100));

		self.state = RemapperState::Stopped;

		Ok(())
	}

	/// Get the current state of the remapper
	pub fn state(&self) -> &RemapperState {
		&self.state
	}
}

struct ThreadRemapper {
	axis_configs: HashMap<u16, AxisConfig>,
}

impl ThreadRemapper {
	/// Process an input event
	fn process_event(&self, event: InputEvent) -> Option<InputEvent> {
		match event.event_type() {
			Some(EventType::EV_ABS) => {
				// if let Some(config) = self.axis_configs.get(code) {
				// 	todo!("apply sensitivity curve with config")
				// }
				let code = event.event_code;
				if code == EventCode::EV_ABS(EV_ABS::ABS_X)
					|| code == EventCode::EV_ABS(EV_ABS::ABS_Y)
					|| code == EventCode::EV_ABS(EV_ABS::ABS_RZ)
				{
					let modified_value = self.apply_sensitivity_curve(event.value);
					eprintln!("Absolute event: {:?} -> {:?}", event, modified_value);
					Some(InputEvent::new(&event.time, &code, modified_value))
				} else {
					Some(event)
				}
			}
			Some(EventType::EV_SYN | EventType::EV_FF | EventType::EV_FF_STATUS) => Some(event),
			None => None,
			Some(_) => Some(event),
		}
	}

	fn apply_sensitivity_curve(&self, value: i32) -> i32 {
		// FIXME: this needs to care about axis config to determine range
		// Convert input range 0->65535 to -1->1
		let normalized = (value as f64 - 32767.5) / 32767.5;

		if normalized > -0.01 && normalized < 0.01 {
			return 32767;
		}

		// Square the normalized value while preserving sign
		let squared = normalized.abs().powi(2) * normalized.signum();

		// Convert back to 0->65535 range
		let result = (squared * 32767.5 + 32767.5) as i32;

		// Clamp to valid range
		result.clamp(0, 65535)
	}
}

fn main() -> Result<()> {
	let devices = DeviceInfo::obtain_device_list()?;
	eprintln!("Devices: {:#?}", devices);
	let device = DeviceInfo::with_name("Thrustmaster Solaris Base", None, Some((1103, 1058, 273)))?;
	eprintln!("Device: {:#?}", device);

	// FIXME: config to select device based on name or VID/PID etc
	let mut remapper = FlightstickCurveApp::new(device.path);

	// FIXME: this needs to be configurable
	remapper.configure_axis(0, AxisConfig { nurbs_curve: None });

	remapper.start()?;

	println!("Remapper is running. Press Enter to stop...");

	// Wait for Enter key
	let mut buffer = [0; 1];
	std::io::stdin().read_exact(&mut buffer)?;

	remapper.stop()?;

	println!("Remapper stopped");

	Ok(())
}
