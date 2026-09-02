![Logo](docs/lotusim_logo.svg)

![Different drones in LOTUSim.](docs/lotusim_environment.png)

LOTUSim is a real-time, multi-domain simulation platform for maritime operations. It models realistic surface, underwater, and air physics for aerial drones, surface ships, and underwater vehicles. An immersive interface lets human operators run human-autonomous agent scenarios, and physically accurate models make LOTUSim suitable for training AI algorithms.

## Quickstart

You need to have Nix setup on your machine (compatible with Linux/macOS). You can install it via this command:
```sh
curl --proto '=https' --tlsv1.2 -L https://nixos.org/nix/install | sh -s -- --daemon
```
For more options, check their official website: [Nix](https://nixos.org/download/). 

#### 1. Add the binary caches

Needs root, once per machine. These let Nix fetch the Gazebo stack and the physics engine prebuilt, saving about an hour of compiling.

```sh
sudo tee -a /etc/nix/nix.conf <<'EOF'
extra-substituters = https://ros.cachix.org https://naval-group.cachix.org
extra-trusted-public-keys = ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo= naval-group.cachix.org-1:ytTEzFEeuzQrC9IRYLzHGa5OnM65G95M6/sbPd0fy28=
EOF
sudo systemctl restart nix-daemon   # or your init system's equivalent
```

`/etc/nix/nix.conf` is the system-wide file, so the keys are trusted for every user and the signatures verify.

#### 2. Install and run:

```sh
nix profile add github:naval-group/LOTUSim github:naval-group/LOTUSim#ui

lotusim run --gui        # the simulation, in a Gazebo window
lotusim-ui               # the browser interface, on http://localhost:8080
```

Away from NixOS, the window needs a GPU bridge — install it once and `lotusim` finds it by itself:

```sh
nix profile add github:nix-community/nixGL#nixGLIntel
```

`lotusim --help` lists the worlds a build carries.

### Without installing anything

```sh
nix run github:naval-group/LOTUSim -- run --gui
podman run --rm ghcr.io/naval-group/lotusim
```

The [wiki](https://github.com/naval-group/LOTUSim/wiki) covers worlds and scenarios, the state directory, NVIDIA and hybrid GPUs, and running under a container.

<br>
<p align="center">
  <strong>⚡ Jump straight to install ➜ running in under 10 minutes: <a href="https://github.com/naval-group/LOTUSim/wiki/getting-started">Getting Started</a></strong>
</p>
<br>

For full documentation, see the [wiki](https://github.com/naval-group/LOTUSim/wiki). For issues or questions, please open an issue and we will get back to you asap.

For partnerships or contributing, contact [lotusim_support@naval-group.com](mailto:lotusim_support@naval-group.com).

Published under [EPL-2.0](LICENSE).

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
