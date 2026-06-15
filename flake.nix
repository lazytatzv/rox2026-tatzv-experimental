{
  description = "Development Environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, utils }:
    utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        
        commonTools = with pkgs; [
          # Override docker cli
          docker
          docker-compose

          # Development tools
          gnumake
          gh
          git
          python3
          python3Packages.black

          ripgrep
          pciutils # For lspci to check GPU hardware
        ];

        guiTools = with pkgs; [
          xorg.xhost
          libnotify
        ];

      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = commonTools ++ guiTools;

          shellHook = ''
            echo "Environment Active"

            # Allow Docker containers to connect to the X server
            xhost +local:docker > /dev/null 2>&1

            export ROS_DOMAIN_ID=0
            export DOCKER_BUILDKIT=1

            # Check for NVIDIA GPU
            if command -v nvidia-smi &> /dev/null; then
              echo "▶ NVIDIA GPU Detected:"
              nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | sed 's/^/  - /'
            else
              echo "▶ WARNING: nvidia-smi not found. GPU acceleration in Docker may not work."
            fi

            # make help
          '';
        };
      }
    );
}
