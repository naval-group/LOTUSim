![Logo](docs/lotusim_logo.svg)

[![LOTUSim Video - IROS2026](https://img.youtube.com/vi/iXDz8ZqSpq4/0.jpg)](https://www.youtube.com/watch?v=iXDz8ZqSpq4)

LOTUSim is a real-time, multi-domain simulation platform for maritime operations. It models realistic surface, underwater, and air physics for aerial drones, surface ships, and underwater vehicles. An immersive interface lets human operators run human-autonomous agent scenarios, and physically accurate models make LOTUSim suitable for training AI algorithms.

## Repositories

| Repository | Description |
|---|---|
| [LOTUSim](https://github.com/naval-group/LOTUSim) | Core simulation engine |
| [LOTUSim-UI-frontend](https://github.com/naval-group/LOTUSim-UI-frontend) | 2D map interface for visualising and creating scenarios |
| [LOTUSim-UI-backend](https://github.com/naval-group/LOTUSim-UI-backend) | Backend supporting the LOTUSim UI |
| [lxdyn](https://github.com/naval-group/lxdyn) | Underwater and surface physics engine |
| [LOTUSim-Unity-modules](https://github.com/naval-group/LOTUSim-Unity-modules)| 3D rendering for LOTUSim scenarios |

## Installation

The [Quickstart](#quickstart) gets you up and running with LOTUSim in about 10 minutes.

For all installation options, see the [Getting Started](https://github.com/naval-group/LOTUSim/wiki/getting-started) guide:

| I want to... | Use this path |
|---|---|
| Try LOTUSim without installing anything | [Path A - Run without installing](https://github.com/naval-group/LOTUSim/wiki/getting-started#path-a---run-without-installing) |
| Install LOTUSim for regular use | [Path B - Install LOTUSim](https://github.com/naval-group/LOTUSim/wiki/getting-started#path-b---install-lotusim) |
| Develop or contribute to LOTUSim | [Path C - Set up your dev environment](https://github.com/naval-group/LOTUSim/wiki/getting-started#path-c---set-up-your-dev-environment) |

For tutorials, models, sensors, batteries, and the Developer Guide, see the [wiki](https://github.com/naval-group/LOTUSim/wiki).

## Quickstart

> **On Windows?** Window users need to first setup WSL2 before going through the steps below. Check out this section in the wiki for the setup steps: [Windows users](https://github.com/naval-group/LOTUSim/wiki/getting-started#windows-users).

#### Step 1 - Install Nix

LOTUSim is installed and run through **Nix**, a package manager. It won't conflict with or change anything else you have installed. You don't need to know Nix to use LOTUSim. Open a terminal and run:

```sh
curl --proto '=https' --tlsv1.2 -L https://nixos.org/nix/install | sh -s -- --daemon
```

Once it finishes, **close your terminal window and open a new one** so the changes take effect. For more options, check their official website: [Nix website](https://nixos.org/download/).

#### Step 2 - Add the binary caches

LOTUSim depends on Gazebo/ROS packages, and on Naval Group's own builds, that aren't part of Nix's normal download cache. Without this step, the very first time you build or run LOTUSim, Nix will quietly compile those packages from scratch, that can take **about an hour**, with no progress message explaining why. It needs admin rights, only once per machine:

```sh
sudo tee -a /etc/nix/nix.conf <<'EOF'
experimental-features = nix-command flakes
extra-substituters = https://ros.cachix.org https://naval-group.cachix.org
extra-trusted-public-keys = ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo= naval-group.cachix.org-1:ytTEzFEeuzQrC9IRYLzHGa5OnM65G95M6/sbPd0fy28=
EOF
sudo systemctl restart nix-daemon   # macOS: sudo launchctl kickstart -k system/org.nixos.nix-daemon
```

> `/etc/nix/nix.conf` is the system-wide file, so the keys are trusted for every user and the signatures verify..

#### Step 3 - Install and run LOTUSim

LOTUSim's 3D window needs to talk to your graphics card. Without this bridge, the simulation window may not open. Install the one matching your graphics card:
```sh
nix profile add github:nix-community/nixGL#nixGLIntel

nix profile add github:naval-group/LOTUSim github:naval-group/LOTUSim#ui
```

On an NVIDIA or hybrid/Optimus machine, also add the NVIDIA bridge so rendering uses the discrete GPU instead of falling back to Intel, this reads your driver's exact version off the running machine, so it needs `--impure`, and NVIDIA's userspace driver is unfree:

```sh
NIXPKGS_ALLOW_UNFREE=1 nix profile add --impure github:nix-community/nixGL#nixGLNvidia
```

You're now ready to run LOTUSim!
```sh
lotusim run --gui        # the simulation, in a Gazebo window
lotusim-ui               # the browser interface, on http://localhost:8080
```

#### Without installing anything
```sh
nix run github:naval-group/LOTUSim -- run --gui
podman run --rm ghcr.io/naval-group/lotusim
```

## Next steps

Want to see more? Check the [Tutorial page](https://github.com/naval-group/LOTUSim/wiki/Tutorial) for further examples, or run `lotusim --help` to see every scenario/world your build includes.

For full documentation, see the [wiki](https://github.com/naval-group/LOTUSim/wiki).

## Support and contact

For issues or questions, please open an issue and we will get back to you asap.

For partnerships or contributing, contact [lotusim_support@naval-group.com](mailto:lotusim_support@naval-group.com).

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
