{
  description = "ROX2026 Development Environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, utils }:
    utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        
        commonTools = with pkgs; [
          docker
          docker-compose
          gnumake
          gh
          git
          python3
          python3Packages.black
          ripgrep
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
            echo "▶ ROX2026 Environment Active"

            # Allow Docker containers to connect to the X server
            xhost +local:docker > /dev/null 2>&1

            alias dc='docker compose'
            
            export ROS_DOMAIN_ID=0
            export DOCKER_BUILDKIT=1

            make help
          '';
        };
      }
    );
}
