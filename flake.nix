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
          pkgs.clang-tools
          # docs/Doxyfile sets HAVE_DOT with an empty DOT_PATH, so doxygen
          # resolves graphviz's dot from PATH.
          pkgs.graphviz
        ];

        # colcon drives the whole workspace in one derivation rather than one
        # derivation per ROS package: the 17 packages share a single CMake
        # invocation order that colcon already knows how to compute.
        # What actually reaches CMake is ~1 MB of sources under systems/,
        # interfaces/ and examples/. Feeding the whole tree in would
        # copy assets/ (160 MB) into the store and make a
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
            !(builtins.elem top [ "assets" "docs" "scripts" ".github" ])
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

        # Store assets are an immutable seed; the UI writes scenarios and uploaded models at runtime.
        seedState = pkgs.writeShellScript "lotusim-seed-state" ''
          set -eu
          export PATH="${pkgs.coreutils}/bin:$PATH"
          state="$1"
          mkdir -p "$state/models" "$state/scenarios" "$state/worlds"

          # A real directory per model with its contents symlinked: uploads need a writable dir, and
          # the UI lists models with a readdir isDirectory() filter that a bare symlink would fail.
          for src in ${assets}/models/*/; do
            dst="$state/models/$(basename "$src")"
            mkdir -p "$dst"
            for child in "$src"*; do
              if [ ! -e "$child" ]; then continue; fi
              link="$dst/$(basename "$child")"
              # Replace only our own symlink, so an uploaded file of the same name survives.
              if [ -L "$link" ] || [ ! -e "$link" ]; then ln -sfn "$child" "$link"; fi
            done
            # An assets bump moves the store path, so drop links this revision no longer provides.
            for link in "$dst"/*; do
              if [ -L "$link" ] && [ ! -e "$link" ]; then rm -f "$link"; fi
            done
          done

          # Real copies: the UI rewrites scenarios in place, and a copied store file keeps mode 444 without the chmod.
          for kind in scenarios worlds; do
            for src in ${assets}/"$kind"/*; do
              if [ ! -e "$src" ]; then continue; fi
              dst="$state/$kind/$(basename "$src")"
              if [ -e "$dst" ]; then continue; fi
              cp -r "$src" "$dst"
              chmod -R u+w "$dst"
            done
          done
        '';

        # Every wrapper that touches assets runs this, so the sim and the UI agree on one location.
        stateHook = ''
          LOTUSIM_STATE_HOME="''${LOTUSIM_STATE_HOME:-''${XDG_DATA_HOME:-$HOME/.local/share}/lotusim}"
          export LOTUSIM_STATE_HOME
          ${seedState} "$LOTUSIM_STATE_HOME"
          export LOTUSIM_MODELS_PATH="''${LOTUSIM_MODELS_PATH:-$LOTUSIM_STATE_HOME/models/}"
          export LOTUSIM_SCENARIOS_PATH="''${LOTUSIM_SCENARIOS_PATH:-$LOTUSIM_STATE_HOME/scenarios}"
          export GZ_SIM_RESOURCE_PATH="$LOTUSIM_STATE_HOME:$LOTUSIM_STATE_HOME/models:${assets}:${assets}/models''${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
        '';

        # mkBackend defaults its two paths to the store, so the hook exports over them before the server starts.
        withState = name: program: pkgs.runCommand name { nativeBuildInputs = [ pkgs.makeWrapper ]; } ''
          makeWrapper ${program} $out/bin/${name} --run ${pkgs.lib.escapeShellArg stateHook}
        '';

        # One implementation of the GPU-driver lookup, shared with the mise task.
        glWrapper = builtins.path {
          path = self + "/scripts/gl-wrapper.sh";
          name = "lotusim-gl-wrapper.sh";
        };

        # xdyn, the physics server each vessel talks to over a websocket. Taken from
        # lxdyn's published deploy image, which is FROM scratch: the layers hold the
        # static binaries and nothing else. Pinned by digest — :latest moves.
        lxdynImage = pkgs.dockerTools.pullImage {
          imageName = "ghcr.io/naval-group/lxdyn";
          imageDigest = "sha256:5361ec8eb091d881a34b49a5015a801a42736dd48be4d33cac5f26b398dc06a0";
          sha256 = "sha256-HYEGXGR6vKnfSGFYsJe5t3jF3whz5PR9q/zTTiPikCE=";
          finalImageTag = "26.8.1";
        };

        xdyn = pkgs.runCommand "xdyn-26.8.1"
          {
            nativeBuildInputs = [ pkgs.jq ];
            # Static musl: no interpreter for patchelf to rewrite, and the archives are
            # the product.
            dontFixup = true;
          } ''
          mkdir -p image rootfs
          tar -xf ${lxdynImage} -C image
          for layer in $(jq -r '.[0].Layers[]' image/manifest.json); do
            tar -xf "image/$layer" -C rootfs
          done
          # Three of the six, not the directory: the image also ships an xdyn tool named
          # `gz`, which would shadow Gazebo's on PATH.
          install -Dm755 -t $out/bin \
            rootfs/usr/bin/xdyn rootfs/usr/bin/xdyn-for-cs rootfs/usr/bin/xdyn-for-me
        '';

        # The ROS and Gazebo setup hooks assemble GZ_CONFIG_PATH,
        # AMENT_PREFIX_PATH, LD_LIBRARY_PATH and PYTHONPATH out of 11 to 133
        # store paths each. Capturing them from a derivation that has the same
        # inputs is exact; writing them out by hand would drift on every bump.
        lotusim-env = pkgs.runCommand "lotusim-env"
          {
            nativeBuildInputs = [ pkgs.makeWrapper ];
            buildInputs = rosDeps ++ gazeboHarmonic ++ thirdParty;
            dontWrapQtApps = true;
          } ''
          # LD_LIBRARY_PATH composes: nixGL exports its mesa into it before exec-ing this.
          makeWrapper ${ros.gz-tools-vendor}/bin/gz $out/bin/lotusim-env \
            --add-flags sim \
            --set GZ_CONFIG_PATH "$GZ_CONFIG_PATH" \
            --set PYTHONPATH "$PYTHONPATH" \
            --set AMENT_PREFIX_PATH "${workspace}:$AMENT_PREFIX_PATH" \
            --prefix LD_LIBRARY_PATH : "${workspace}/lib:$LD_LIBRARY_PATH" \
            --set GZ_SIM_SYSTEM_PLUGIN_PATH "${workspace}/lib" \
            --set FASTDDS_BUILTIN_TRANSPORTS UDPv4
        '';

        # `run` is the shape the old shell script had, kept because it says what it
        # means; anything starting with a dash still goes straight to gz sim, which
        # is what the container's default command relies on.
        lotusim = pkgs.writeShellApplication {
          name = "lotusim";
          runtimeInputs = [ pkgs.coreutils ];
          text = ''
            ${stateHook}

            usage() {
              cat <<USAGE
            lotusim — the LOTUSim simulation server (Gazebo Harmonic)

            Usage:
              lotusim run [--gui] [--debug] [<world>]    world defaults to lotusim.world
              lotusim --help

            Worlds are taken from $LOTUSIM_STATE_HOME/worlds first, then from this build:
            $(for w in ${assets}/worlds/*.world; do echo "  $(basename "$w")"; done)

            Scenarios you create and models you upload are written to
              $LOTUSIM_STATE_HOME
            LOTUSIM_STATE_HOME moves all of it; GZ_SIM_RESOURCE_PATH,
            LOTUSIM_MODELS_PATH and LOTUSIM_SCENARIOS_PATH override one at a time.

            The web UI is its own entry point:
              nix run github:naval-group/LOTUSim#ui      http://localhost:8080

            An argument starting with a dash goes straight to gz sim, options and all.
            USAGE
            }

            # A nix-built binary cannot reach a non-NixOS host's GPU driver. The
            # window needs it, and so does every rendering sensor — camera,
            # gpu_lidar — which gz implements by rendering the scene.
            glwrap=()
            gl_bridge() {
              glwrap=()
              local wrapper
              if wrapper=$(${pkgs.bash}/bin/bash ${glWrapper}); then
                if [ -n "$wrapper" ]; then glwrap=("$wrapper"); fi
                return 0
              fi
              return 1
            }

            no_bridge_warning="warning: no GPU driver bridge — camera and gpu_lidar sensors will not render."

            case "''${1:-}" in
              "" | -h | --help)
                usage
                exit 0
                ;;
              ui)
                echo "The UI is its own entry point, and is not in this command's closure:" >&2
                echo "  nix run github:naval-group/LOTUSim#ui" >&2
                exit 1
                ;;
              build)
                echo "Building needs a clone: nix develop, then mise run build." >&2
                exit 1
                ;;
              run)
                shift
                ;;
              -*)
                gl_bridge || echo "$no_bridge_warning" >&2
                exec "''${glwrap[@]}" ${lotusim-env}/bin/lotusim-env "$@"
                ;;
              *)
                echo "lotusim: unknown command '$1'" >&2
                usage >&2
                exit 1
                ;;
            esac

            gui=false
            debug=false
            while [ $# -gt 0 ]; do
              case "$1" in
                --gui) gui=true; shift ;;
                --debug) debug=true; shift ;;
                -*) echo "lotusim run: unknown option '$1'" >&2; exit 1 ;;
                *) break ;;
              esac
            done

            world="''${1:-lotusim.world}"
            world_file=""
            for root in "$LOTUSIM_STATE_HOME" "${assets}"; do
              if [ -f "$root/worlds/$world" ]; then
                world_file="$root/worlds/$world"
                break
              fi
            done
            if [ -z "$world_file" ]; then
              echo "lotusim: world '$world' is in neither $LOTUSIM_STATE_HOME/worlds nor this build." >&2
              exit 1
            fi

            if [ "$gui" = true ]; then
              if [ -z "''${WAYLAND_DISPLAY:-}" ] && [ -z "''${DISPLAY:-}" ]; then
                echo "--gui needs a graphical session: neither WAYLAND_DISPLAY nor DISPLAY is set." >&2
                exit 1
              fi
              headless=()
            else
              # gz sim -s is server-only; dropping it is what opens the window.
              headless=(-s)
            fi

            if [ "$debug" = true ]; then
              verbosity=-v4
              export LOTUSIM_SPDLOG_LEVEL=debug
            else
              verbosity=-v1
            fi

            if ! gl_bridge; then
              if [ "$gui" = true ]; then
                # gz exits 0 when the GUI aborts, so refuse now.
                echo "No GPU driver bridge — the GUI cannot reach a driver." >&2
                echo "  nix profile add github:nix-community/nixGL#nixGLIntel" >&2
                exit 1
              fi
              echo "$no_bridge_warning" >&2
            fi

            exec "''${glwrap[@]}" ${lotusim-env}/bin/lotusim-env \
              "$verbosity" "''${headless[@]}" -r "$world_file"
          '';
        };

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
            Cmd = [ "run" ];
          };
        };

        ui-backend-unwrapped = lotusim-ui-backend.lib.mkBackend {
          inherit pkgs assets;
          rosMessages = messages;
        };

        ui-backend = withState "lotusim-ui-backend" "${ui-backend-unwrapped}/bin/lotusim-ui-backend";

        ui-frontend = lotusim-ui-frontend.packages.${system}.default;

        # wait -n returns on the first exit; the trap takes the other half down with it.
        ui = pkgs.writeShellApplication {
          name = "lotusim-ui";
          text = ''
            # The backend announces its own :5000; say which half to open first.
            echo "LOTUSim UI"
            echo "  open http://localhost:8080   (the backend serves the API on :5000)"
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
          packages = tooling ++ shellTooling ++ rosDeps ++ gazeboHarmonic ++ thirdParty ++ [ xdyn ];

          # colcon defaults to make; ninja is what gets the workspace to ~2 min.
          shellHook = ''
            export CMAKE_GENERATOR=Ninja

            # mise.toml is the one definition of the project environment; the
            # shell evaluates it so `nix develop` and `mise run` agree, and so
            # the examples find LOTUSIM_* and xdyn-for-cs without a mise prefix.
            mise trust >/dev/null 2>&1 || true
            eval "$(mise env -s bash)"
            LOTUSIM_PATH="''${LOTUSIM_PATH:-$PWD}"

            # What `source install/setup.bash` sets, exported up front so the
            # examples run straight after `mise run build`. A directory that
            # does not exist yet is harmless on any of these.
            export COLCON_PREFIX_PATH="$LOTUSIM_PATH/install"
            export AMENT_PREFIX_PATH="$LOTUSIM_PATH/install''${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
            export CMAKE_PREFIX_PATH="$LOTUSIM_PATH/install''${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
            export LD_LIBRARY_PATH="$LOTUSIM_PATH/install/lib''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
            export PYTHONPATH="$LOTUSIM_PATH/install/${pkgs.python3.sitePackages}''${PYTHONPATH:+:$PYTHONPATH}"

            echo "LOTUSim devShell — ROS 2 jazzy / Gazebo Harmonic"
          '';
        };
      });
}
