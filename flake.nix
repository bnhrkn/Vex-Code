{
  description = "Dev shell for gcc-arm-none-eabi and pros-cli";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-21.11"; # Adjust this to your preferred channel or pinning
    flake-utils.url = "github:numtide/flake-utils";
    mach-nix.url = "github:DavHau/mach-nix?ref=3.3.0"; # Adjust the version if necessary
  };

  outputs = { self, nixpkgs, flake-utils, mach-nix }:
    flake-utils.lib.eachDefaultSystem (system:
      let
       
        pkgs = import nixpkgs { inherit system; };

        pythonEnv = (import mach-nix {}).mkPython {
          # This is a minimal set of python packages, adjust as needed
          requirements = ''
            pros-cli
          '';

          providers = {
            pros-cli = "sdist";
          };
        };
      in
      {
        devShell = pkgs.mkShell {
          buildInputs = [
            pkgs.gcc-arm-embedded
            pythonEnv
          ];

          shellHook = ''
            echo "Development shell for gcc-arm-none-eabi and pros-cli is ready!"
          '';
        };
      }
    );
}