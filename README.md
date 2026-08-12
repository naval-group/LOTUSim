![Logo](docs/lotusim_logo.svg)

![Different drones in LOTUSim.](docs/lotusim_environment.png)

LOTUSim is a real-time, multi-domain simulation platform for maritime operations. It models realistic surface, underwater, and air physics for aerial drones, surface ships, and underwater vehicles. An immersive interface lets human operators run human-autonomous agent scenarios, and physically accurate models make LOTUSim suitable for training AI algorithms.

<br>
<p align="center">
  <strong>⚡ Jump straight to install ➜ running in under 10 minutes: <a href="https://github.com/naval-group/LOTUSim/wiki/getting-started">Getting Started</a></strong>
</p>
<br>

For full documentation, see the [wiki](https://github.com/naval-group/LOTUSim/wiki). For issues or questions, please open an issue and we will get back to you asap.

For partnerships or contributing, contact [lotusim_support@naval-group.com](mailto:lotusim_support@naval-group.com).

Published under [EPL-2.0](LICENSE).

## Using LOTUSim

### With nix

Installs a `lotusim` command on your PATH:

```sh
nix profile install github:naval-group/LOTUSim
lotusim -s -r worlds/lotusim.world
```

Worlds and models are bundled, so `worlds/…` resolves without a full path. To use your own, set `GZ_SIM_RESOURCE_PATH` and `LOTUSIM_MODELS_PATH` — the wrapper defers to both when they are already set.

### With podman

Installs nothing:

```sh
podman run --rm ghcr.io/naval-group/lotusim
podman run --rm ghcr.io/naval-group/lotusim -s -r worlds/other.world
```

Passing arguments replaces the image's default command, so the world has to be named again.

LOTUSim talks to Unity on `:23456`/`:23457` and to xdyn over a websocket. 
Under nix those are ordinary localhost connections; under podman each one needs `-p` publishing, or `--network=host`. 

## Developing for LOTUSim

Everything below is for developing LOTUSim. The routes above are all you need to run it.

The development environment is a Nix devShell, so the ROS 2 and Gazebo versions are a property of this repository rather than of the host distribution. 
Any machine with [Nix](https://nixos.org/download/) works.

### Step 1 — Configure the ROS binary cache

The Gazebo Harmonic vendor packages are not in `cache.nixos.org`. Without this step Nix builds roughly 370 derivations from source (about one hour) and prints neither an error nor a warning. It needs root, once per machine:

```sh
sudo tee -a /etc/nix/nix.conf <<'EOF'
extra-substituters = https://ros.cachix.org
extra-trusted-public-keys = ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=
EOF
sudo systemctl restart nix-daemon   # or your init system's equivalent
```

The `nixConfig` block in `flake.nix` requests the same cache, but Nix applies it only to users listed in `trusted-users` — for everyone else it prints `ignoring untrusted flake configuration setting`, which is why this step is not optional. A single-user install has no daemon and no `trusted-users`, so it can skip this step entirely.

### Step 2 — Clone the repository

```sh
git clone https://github.com/naval-group/LOTUSim.git
cd LOTUSim
```

### Step 3 — Enter the environment and build

```sh
nix develop          # ROS 2 jazzy + Gazebo Harmonic + colcon + mise
mise run build       # colcon build --merge-install
```

`mise tasks` lists everything available.

This builds into `install/` for development; it does not put `lotusim` on your PATH. 
To install what you have just changed, use `nix profile install .#lotusim`.

### Step 4 — Run a simulation

```sh
mise run sim                       # headless, the default world
mise run sim --gui                 # opens the Gazebo window
mise run sim aerialWorld.world     # any file in worlds/ of any assets root
```

`--assets-path` adds assets roots to the core one rather than replacing it, so a consumer project composes its own models and worlds with the core catalogue. It is colon-separated and repeatable, and both forms append:

```sh
mise run sim --assets-path /path/to/mine other.world
mise run sim --assets-path /a:/b --assets-path /c other.world
```

Roots are searched in order. A world is taken from the first root that holds it; `model://` URIs resolve across all of them. `mise run test:assets` checks this.

Running a world only starts Gazebo. Vessel dynamics come from a separate `xdyn-for-cs` process per vessel, over a websocket — without one, a vessel spawns and renders but does not move.

`mise run check` is the headless smoke test: it spawns vessels and fails if gz logs an error. It needs no xdyn, because gz exits 0 whether or not a model resolved, so the log is the only signal.

The Gazebo GUI needs the host's GPU driver, which a nix-built binary cannot reach on a non-NixOS host. Install [nixGL](https://github.com/nix-community/nixGL) once into your own profile and `mise run sim --gui` finds it automatically, including inside `nix develop`, which does not put the profile on `PATH`:

```sh
nix profile install github:nix-community/nixGL#nixGLIntel
```

That variant covers Intel and AMD, and is also the right choice on a hybrid Intel/NVIDIA laptop, where the GUI runs under both Wayland and Xwayland. NVIDIA-only machines need `nixGLNvidia`, which has to match the host's driver version and so installs with `--impure` — which is also why nixGL cannot be a dependency of this repository: it must match the machine, not the project. Set `LOTUSIM_GL_WRAPPER` to a name or path to override the choice. On NixOS none of this applies.

If the GUI aborts inside Ogre2's EGL setup, the renderer has fallen back from GLX and chosen a GPU by enumeration order, which on a hybrid machine can be the NVIDIA device that nix's Mesa cannot drive against the proprietary `nvidia-drm` module. Run `mise run sim --gui --debug` to see the fallback. `LIBGL_ALWAYS_SOFTWARE=1` renders on the CPU and always works; masking the unwanted `/dev/dri` nodes with `bwrap` forces the other GPU.

### Optional: the web UI

The UI lives in two repositories of its own, [LOTUSim-UI-backend](https://github.com/naval-group/LOTUSim-UI-backend) and [LOTUSim-UI-frontend](https://github.com/naval-group/LOTUSim-UI-frontend), each packaged as its own flake. This flake composes them, because the backend's ROS message bindings are generated from `interfaces/` and only LOTUSim can supply them.

**If you are not changing the UI, you need neither checkout.** Run it the way a user does:

```sh
nix run .#ui                 # backend :5000 and frontend :8080
nix run .#ui-backend         # or either half alone
nix run .#ui-frontend
```

Editing `interfaces/` is enough to change what the UI can talk about: `packages.messages` builds from your working tree, so the next `nix run .#ui` carries the new types.

**If you are changing the UI**, clone only the half you are editing and take the other from this flake. The frontend links no ROS, so its own devShell is enough:

```sh
git clone https://github.com/naval-group/LOTUSim-UI-frontend.git
cd LOTUSim-UI-frontend
nix develop                                  # nodejs
npm ci && npm run dev                        # vite, http://localhost:5173
nix run /path/to/LOTUSim#ui-backend          # the other half
```

The backend's shell has to come from this repository, because only LOTUSim generates its messages:

```sh
git clone https://github.com/naval-group/LOTUSim-UI-backend.git
cd LOTUSim-UI-backend
nix develop /path/to/LOTUSim#ui-backend      # node, ROS, and lotusim_msgs on AMENT_PREFIX_PATH
npm run setup                                # rclnodejs ships no prebuilt for this Node, so it compiles
npx generate-ros-messages
npm run dev                                  # http://localhost:5000
nix run /path/to/LOTUSim#ui-frontend         # the other half
```

Vite serves on `:5173` while the packaged frontend serves on `:8080`; both reach the backend on `:5000`. Before pushing, check the change still works in the packaged build, which is what users actually get:

```sh
nix run .#ui --override-input lotusim-ui-frontend git+file:///path/to/LOTUSim-UI-frontend
```

### Optional: container image

The image is built by the flake, not by a Dockerfile:

```sh
mise run image               # nix build .#container, then load into podman or docker
podman run --rm lotusim:latest                              # the default world
podman run --rm lotusim:latest -s -r worlds/other.world     # any world in assets/
```

This builds the image locally; the published one is in [Using LOTUSim](#with-podman).

## Video

[![LOTUSim Video - IROS2026](https://img.youtube.com/vi/iXDz8ZqSpq4/0.jpg)](https://www.youtube.com/watch?v=iXDz8ZqSpq4)

## Citation

If you use LOTUSim in your research, please cite:

```bibtex
@inproceedings{LOTUSim26iros,
  title     = {{LOTUSim}: Multi-Domain Simulator for Marine Robotics},
  author    = {Buche, Cedric and Grosset, Juliette and Lechene, Helene and Dubromel, Marie and Havez-Bodivit, Pierig and Neo, Malcom and Prodhon, Julien},
  booktitle = {2026 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2026},
  publisher = {IEEE}
}
```

See the [Publications](https://github.com/naval-group/LOTUSim/wiki/Publications) wiki page for related repositories and papers.
