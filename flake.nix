{
  description = "LOTUSim — ROS 2 jazzy + Gazebo Harmonic development environment";

  # The Gazebo Harmonic vendor stack is absent from cache.nixos.org. Nix honours
  # this only for trusted-users; everyone else needs the substituter in
  # /etc/nix/nix.conf, or the shell silently builds ~370 derivations from source.
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
    ];
  };

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    flake-utils.follows = "nix-ros-overlay/flake-utils";

    # Arrows point one way: the UI exports builders, this flake passes its messages in.
    lotusim-ui-backend = {
      url = "github:naval-group/LOTUSim-UI-backend";
      inputs.nix-ros-overlay.follows = "nix-ros-overlay";
    };

    # The frontend links against no ROS, so nixpkgs alone is enough to dedup.
    lotusim-ui-frontend = {
      url = "github:naval-group/LOTUSim-UI-frontend";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
  };

  outputs = { self, nix-ros-overlay, nixpkgs, flake-utils, lotusim-ui-backend, lotusim-ui-frontend }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        ros = pkgs.rosPackages.jazzy;

        # The nine Harmonic libraries LOTUSim's CMakeLists find_package directly.
        # They resolve through the overlay's *-vendor packages, not nixpkgs proper.
        gazeboHarmonic = with ros; [
          gz-cmake-vendor
          gz-common-vendor
          gz-math-vendor
          gz-msgs-vendor
          gz-plugin-vendor
          gz-rendering-vendor
          gz-sensors-vendor
          gz-sim-vendor
          gz-transport-vendor
          gz-utils-vendor
          sdformat-vendor
          # pulled in transitively but named here so a drop is loud, not silent
          gz-fuel-tools-vendor
          gz-tools-vendor
        ];

        rosDeps = with ros; [
          ament-cmake
          ament-cmake-gtest
          ament-lint-auto
          ament-lint-common
          action-msgs
          backward-ros
          builtin-interfaces
          geographic-msgs
          geometry-msgs
          radar-msgs
          rclcpp
          rclcpp-action
          rosidl-default-generators
          rosidl-default-runtime
          sensor-msgs
          std-msgs
          std-srvs
        ];

        # Boost comes from the overlay, not nixpkgs: Gazebo builds against the
        # overlay's 1.89.0, and pulling nixpkgs' would put a second identical
        # version in the closure for ~170 MB.
        thirdParty = [ ros.boost ] ++ (with pkgs; [
          eigen
          nlohmann_json
          readline
          spdlog
          websocketpp
          yaml-cpp
        ]);

        # What the workspace derivation builds with, and so also what it carries
        # in its build closure.
        tooling = [
          ros.ros-core
          ros.ros2cli
          pkgs.colcon
          pkgs.cmake
          pkgs.ninja
          pkgs.pkg-config
          pkgs.python3
        ];

        # Only the shell needs these. Keeping them out of tooling keeps them out
        # of the workspace's build closure, and stops a change here from
        # invalidating the build.
        shellTooling = [
          pkgs.mise
          pkgs.nodejs_22
          pkgs.doxygen
          # docs/Doxyfile sets HAVE_DOT with an empty DOT_PATH, so doxygen
          # resolves graphviz's dot from PATH.
          pkgs.graphviz
        ];

        # colcon drives the whole workspace in one derivation rather than one
        # derivation per ROS package: the 17 packages share a single CMake
        # invocation order that colcon already knows how to compute.
        # What actually reaches CMake is ~1 MB of sources under systems/,
        # interfaces/, examples/ and launch/. Feeding the whole tree in would
        # copy assets/ (160 MB) and physics/ (42 MB) into the store and make a
        # README edit invalidate a four-minute build. Deny-list rather than
        # allow-list, so a new package directory still builds by default.
        workspaceSrc = pkgs.lib.cleanSourceWith {
          name = "lotusim-workspace";
          src = self;
          filter = path: _type:
            let
              rel = pkgs.lib.removePrefix "${self}/" path;
              top = builtins.head (pkgs.lib.splitString "/" rel);
            in
            !(builtins.elem top [ "assets" "physics" "docs" ".github" ".vscode" ])
            && !(pkgs.lib.hasSuffix ".md" rel)
            && !(builtins.elem rel [ "flake.nix" "flake.lock" "mise.toml" ]);
        };

        colconWorkspace = { pname, src, buildInputs }: pkgs.stdenv.mkDerivation {
          inherit pname src buildInputs;
          version = "0.1.1";

          nativeBuildInputs = tooling;

          # colcon calls cmake itself, once per package.
          dontConfigure = true;

          # gz-sim-vendor drags Qt in for the GUI, but nothing here is a Qt app.
          dontWrapQtApps = true;

          buildPhase = ''
            runHook preBuild
            export HOME=$TMPDIR
            colcon --log-base $TMPDIR/log build \
              --merge-install \
              --install-base $out \
              --build-base $TMPDIR/build \
              --cmake-args -DCMAKE_BUILD_TYPE=Release
            runHook postBuild
          '';

          # colcon has already written everything to $out.
          dontInstall = true;
        };

        workspace = colconWorkspace {
          pname = "lotusim-workspace";
          src = workspaceSrc;
          buildInputs = rosDeps ++ gazeboHarmonic ++ thirdParty;
        };

        # What the UI backend gets: handing it ${workspace} would put 3.02 GB of Gazebo behind it.
        messages = colconWorkspace {
          pname = "lotusim-messages";
          src = builtins.path {
            path = self + "/interfaces";
            name = "lotusim-interfaces";
          };
          buildInputs = with ros; [
            ament-cmake
            action-msgs
            builtin-interfaces
            geographic-msgs
            geometry-msgs
            rosidl-default-generators
            rosidl-default-runtime
            sensor-msgs
            std-msgs
            std-srvs
          ];
        };

        # builtins.path narrows the dependency to assets/, so an edit elsewhere in the tree does not rebuild.
        assets = builtins.path {
          path = self + "/assets";
          name = "lotusim-assets";
        };

        # packages holds derivations only; a symlink keeps .#assets checkable without a second 160 MB copy.
        assetsPackage = pkgs.runCommand "lotusim-assets" { } "ln -s ${assets} $out";

        # The ROS and Gazebo setup hooks assemble GZ_CONFIG_PATH,
        # AMENT_PREFIX_PATH, LD_LIBRARY_PATH and PYTHONPATH out of 11 to 133
        # store paths each. Capturing them from a derivation that has the same
        # inputs is exact; writing them out by hand would drift on every bump.
        lotusim = pkgs.runCommand "lotusim"
          {
            nativeBuildInputs = [ pkgs.makeWrapper ];
            buildInputs = rosDeps ++ gazeboHarmonic ++ thirdParty;
            dontWrapQtApps = true;
          } ''
          makeWrapper ${ros.gz-tools-vendor}/bin/gz $out/bin/lotusim \
            --add-flags sim \
            --set GZ_CONFIG_PATH "$GZ_CONFIG_PATH" \
            --set PYTHONPATH "$PYTHONPATH" \
            --set AMENT_PREFIX_PATH "${workspace}:$AMENT_PREFIX_PATH" \
            --set LD_LIBRARY_PATH "${workspace}/lib:$LD_LIBRARY_PATH" \
            --set GZ_SIM_SYSTEM_PLUGIN_PATH "${workspace}/lib" \
            --prefix GZ_SIM_RESOURCE_PATH : "${assets}:${assets}/models" \
            --set-default LOTUSIM_MODELS_PATH "${assets}/models/" \
            --set-default LOTUSIM_SCENARIOS_PATH "${assets}/scenarios" \
            --set FASTDDS_BUILTIN_TRANSPORTS UDPv4
        '';

        # streamLayeredImage builds a script that writes the image to stdout,
        # so the ~1 GB archive is never materialised on disk — `./result | docker
        # load`. Layering still separates the gz runtime, which is the bulk of
        # the image, from the workspace on top, which changes far more often.
        container = pkgs.dockerTools.streamLayeredImage {
          name = "lotusim";
          tag = "latest";
          contents = [ lotusim pkgs.bashInteractive pkgs.coreutils ];
          config = {
            # gz writes its log under $HOME and warns it cannot without one.
            Env = [ "HOME=/tmp" ];
            WorkingDir = "/tmp";
            Entrypoint = [ "/bin/lotusim" ];
            Cmd = [ "-s" "-r" "${assets}/worlds/lotusim.world" ];
          };
        };

        ui-backend = lotusim-ui-backend.lib.mkBackend {
          inherit pkgs assets;
          rosMessages = messages;
        };

        ui-frontend = lotusim-ui-frontend.packages.${system}.default;

        # wait -n returns on the first exit; the trap takes the other half down with it.
        ui = pkgs.writeShellApplication {
          name = "lotusim-ui";
          text = ''
            "${ui-backend}/bin/lotusim-ui-backend" &
            backend=$!
            "${ui-frontend}/bin/lotusim-ui-frontend" &
            frontend=$!
            trap 'kill "$backend" "$frontend" 2>/dev/null || true' EXIT INT TERM
            wait -n "$backend" "$frontend"
          '';
        };
      in
      {
        packages = {
          inherit lotusim workspace messages container ui-backend ui-frontend ui;
          assets = assetsPackage;
          default = lotusim;
        };

        apps = {
          default = {
            type = "app";
            program = "${lotusim}/bin/lotusim";
          };

          ui-backend = {
            type = "app";
            program = "${ui-backend}/bin/lotusim-ui-backend";
          };

          ui-frontend = {
            type = "app";
            program = "${ui-frontend}/bin/lotusim-ui-frontend";
          };

          ui = {
            type = "app";
            program = "${ui}/bin/lotusim-ui";
          };
        };

        # The backend's own shell, plus the messages only this flake can supply.
        devShells.ui-backend = lotusim-ui-backend.lib.mkBackendShell {
          inherit pkgs;
          rosMessages = messages;
        };

        devShells.default = pkgs.mkShell {
          name = "lotusim";
          packages = tooling ++ shellTooling ++ rosDeps ++ gazeboHarmonic ++ thirdParty;

          # colcon defaults to make; ninja is what gets the workspace to ~2 min.
          shellHook = ''
            export CMAKE_GENERATOR=Ninja
            echo "LOTUSim devShell — ROS 2 jazzy / Gazebo Harmonic"
          '';
        };
      });
}
