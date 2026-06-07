{
  description = "Working env";

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

            # --- System / GUI ---            xorg.xhost
          ];

          shellHook = ''
            echo "--- Environment Loaded via Nix Flakes ---"

            # Auto-setup for Docker GUI if on Linux
            if [ "$(uname)" = "Linux" ]; then
              xhost +local:docker > /dev/null 2>&1
            fi

            # Convenience Aliases
            alias dc='docker compose'

            export ROS_DOMAIN_ID=0
            export DOCKER_BUILDKIT=1

            make help
          '';
        };
      }
    );
}
