{
  description = "Ultimate Working Environment for ROX2026 - Hyprland/Wayland Optimized";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, utils }:
    utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            # --- Docker & Orchestration ---
            docker
            docker-compose
            gnumake

            # --- GitHub & CI/CD ---
            gh
            git

            # --- Python & Utils ---
            python3
            python3Packages.black
            ripgrep

            # --- System / GUI / Hyprland Support ---
            xorg.xhost
            wf-recorder   # Wayland Screen Recorder
            slurp         # Mouse Region Selector
            grim          # Screenshot Tool
            libnotify     # Desktop Notifications
          ];

          shellHook = ''
            echo "--- 🚀 Ultimate Environment Loaded (Hyprland Ready) ---"

            # Auto-setup for Docker GUI if on Linux
            if [ "$(uname)" = "Linux" ]; then
              xhost +local:docker > /dev/null 2>&1
            fi

            # Convenience Aliases
            alias dc='docker compose'
            
            # --- Recording Aliases (Pro level) ---
            # record-region: Select window/region and record to robot_run.mp4
            alias record-region='wf-recorder -g "$(slurp)" -f robot_run.mp4 && notify-send "Recording Saved" "robot_run.mp4 created"'
            
            # record-full: Record full screen
            alias record-full='wf-recorder -f robot_run.mp4'

            export ROS_DOMAIN_ID=0
            export DOCKER_BUILDKIT=1

            make help
          '';
        };
      }
    );
}
