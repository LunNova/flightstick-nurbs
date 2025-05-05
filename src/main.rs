use anyhow::{Context, Result};
use evdev_rs::{
	Device, DeviceWrapper, GrabMode, InputEvent, ReadFlag, ReadStatus, UInputDevice,
	enums::{EV_ABS, EventCode, EventType},
};
use std::{
	collections::HashMap,
	io::Read,
	path::Path,
	sync::{
		Arc,
		atomic::{AtomicBool, Ordering},
	},
	thread,
	time::Duration,
};

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
	source_path: String,
	state: RemapperState,
	running: Arc<AtomicBool>,
	axis_configs: HashMap<u16, AxisConfig>,
}

impl FlightstickCurveApp {
	pub fn new(source_path: &str) -> Self {
		Self {
			source_path: source_path.to_string(),
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
			self.source_path
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
	// FIXME: config to select device based on name or VID/PID etc
	let mut remapper = FlightstickCurveApp::new("/dev/input/event11");

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
