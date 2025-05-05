{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = args:
    args.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = (import args.nixpkgs) {
          inherit system;
        };

        runtimeDeps = [
          pkgs.pkg-config
          pkgs.libevdev
        ];

        LD_LIBRARY_PATH = "/run/opengl-driver/lib/:${pkgs.lib.makeLibraryPath runtimeDeps}";

        devShellPkgs = [
          pkgs.cargo-deny
          pkgs.cargo-bloat
          pkgs.cargo-flamegraph
          pkgs.cargo-udeps
          pkgs.rustfmt
          pkgs.pkg-config
          pkgs.just
          pkgs.cmake
        ] ++ runtimeDeps;

        self = {
          packages.default = self.packages.x86_64-pc-windows-gnu;

          devShells.default = self.devShells.rustup-dev;

          devShells.rustup-dev = pkgs.stdenv.mkDerivation {
            inherit LD_LIBRARY_PATH;
            name = "rustup-dev-shell";

            # need unset for adhoc cargo cross builds in devshell to work
            shellHook = ''
              export CC=
              export NIX_CFLAGS_COMPILE=
              export NIX_CFLAGS_COMPILE_FOR_TARGET=
            '';

            depsBuildBuild = with pkgs; [
              pkg-config
            ];

            nativeBuildInputs = with pkgs; [
              mold
              lld
              bubblewrap
            ];

            GLIBC_PATH = "${pkgs.glibc_multi}/lib";

            buildInputs = with pkgs; [
              glibc_multi
              rustup
              libunwind
              stdenv.cc
            ] ++ devShellPkgs;
          };
        };
      in
      self
    );
}
