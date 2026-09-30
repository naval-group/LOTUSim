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

For all installation options, **including the Docker image**, see the [Getting Started](https://github.com/naval-group/LOTUSim/wiki/getting-started) guide:

| I want to... | Use this path |
|---|---|
| Try LOTUSim without installing anything | [Path A - Run without installing](https://github.com/naval-group/LOTUSim/wiki/getting-started#path-a---run-without-installing) |
| Install LOTUSim for regular use | [Path B - Install LOTUSim](https://github.com/naval-group/LOTUSim/wiki/getting-started#path-b---install-lotusim) |
| Develop or contribute to LOTUSim | [Path C - Set up your dev environment](https://github.com/naval-group/LOTUSim/wiki/getting-started#path-c---set-up-your-dev-environment) |

For tutorials, models, sensors, batteries, and the Developer Guide, see the [wiki](https://github.com/naval-group/LOTUSim/wiki).

## Quickstart

Pick your platform for the one-time environment setup, then continue with [Step 4](#step-4---install-and-run-lotusim).

<details>
<summary><b>Linux / macOS</b></summary>

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

> `/etc/nix/nix.conf` is the system-wide file, so the keys are trusted for every user and the signatures verify.

#### Step 3 - Install a graphics bridge (optional, Linux only)

Programs installed through Nix can't see your system's graphics drivers on their own, so LOTUSim's 3D Gazebo window may fail to open or fall back to slow software rendering. [nixGL](https://github.com/nix-community/nixGL) bridges that gap by pointing LOTUSim at the driver already on your machine. It's optional: pick the bridge that matches your graphics card, or skip it and come back here if `lotusim run --gui` doesn't open a window. macOS users can skip this step.

Not sure which graphics card you have? Run `lspci | grep -iE 'vga|3d'`.

**Intel or AMD** (both use the open-source Mesa drivers, so they share one bridge despite the `Intel` in its name):

```sh
nix profile add github:nix-community/nixGL#nixGLIntel
```

**NVIDIA**, including hybrid/Optimus laptops (for hybrid laptops, install the Intel/AMD bridge above too). It reads your installed driver's exact version, so it needs `--impure`, and NVIDIA's driver is unfree, so it needs `NIXPKGS_ALLOW_UNFREE=1`:

```sh
NIXPKGS_ALLOW_UNFREE=1 nix profile add --impure github:nix-community/nixGL#nixGLNvidia
```

</details>

<details>
<summary><b>Windows (WSL2)</b></summary>

Nix doesn't run natively on Windows, so you'll run LOTUSim inside **NixOS-WSL**, a full NixOS system running under WSL2.

##### 1. Install WSL (skip if you already have it)

Open PowerShell **as Administrator** and run:

```
wsl --install --no-distribution
```

Restart Windows if prompted.

##### 2. Install NixOS-WSL

Download `nixos.wsl` from the [latest NixOS-WSL release](https://github.com/nix-community/NixOS-WSL/releases/latest). If you have WSL 2.4.4 or later, you can double-click the file to install it. Or install it from PowerShell:

```
wsl --install --from-file nixos.wsl
```

On older WSL versions, use:

```
wsl --import NixOS $env:USERPROFILE\NixOS nixos.wsl --version 2
```

Then open it:

```
wsl -d NixOS
```

Optionally, make it your default distro with `wsl -s NixOS`.

##### 3. Enable mirrored networking

This lets ROS 2's device discovery work without extra configuration, and lets your Windows browser reach the LOTUSim UI on `localhost`. In PowerShell, open:

```
notepad $env:USERPROFILE\.wslconfig
```

and add:

```
[wsl2]
networkingMode=mirrored
memory=16GB
swap=16GB
```

##### 4. Configure NixOS for LOTUSim

This step sets nix binary caches and graphics bridge. Inside the NixOS shell, open the system config:

```
sudo nano /etc/nixos/configuration.nix
```

Add these lines inside the main `{ ... }` block. Keep what's already there, especially the `imports` and `system.stateVersion` lines.

```nix
  # Flakes + binary caches (replaces Step 2)
  nix.settings = {
    experimental-features = [ "nix-command" "flakes" ];
    extra-substituters = [ "https://ros.cachix.org" "https://naval-group.cachix.org" ];
    extra-trusted-public-keys = [
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
      "naval-group.cachix.org-1:ytTEzFEeuzQrC9IRYLzHGa5OnM65G95M6/sbPd0fy28="
    ];
  };

  # GPU access through the Windows driver (replaces Step 3)
  hardware.graphics.enable = true;
  wsl.useWindowsDriver = true;

  # Needed to clone the repo and for flakes to see your files
  environment.systemPackages = with pkgs; [ git ];
```

Apply it:

```
sudo nixos-rebuild switch
```

> Without the trusted keys, Nix will quietly build the ROS/Gazebo packages from source the first time, which takes about an hour. If a build seems stuck, check that this rebuild succeeded.

##### 5. Restart WSL

In PowerShell:

```
wsl --shutdown
```

Then reopen NixOS with `wsl -d NixOS` and continue with **Step 4** below.

> **Troubleshooting:** If ROS nodes can't find each other, check the Windows Firewall first, since mirrored networking is sometimes blocked by default. NixOS also has its own firewall; for local development you can add `networking.firewall.enable = false;` to `configuration.nix` and rebuild. If `--gui` fails to open a window, try the Intel/AMD nixGL bridge from Step 3 (in the Linux / macOS section) as a fallback.

</details>

#### Step 4 - Install and run LOTUSim

```sh
nix profile add github:naval-group/LOTUSim github:naval-group/LOTUSim#ui
```

You're now ready to run LOTUSim!
```sh
lotusim run --gui        # the simulation, in a Gazebo window
lotusim-ui               # the browser interface, on http://localhost:8080
```

#### Without installing anything
```sh
nix run github:naval-group/LOTUSim -- run --gui   # the simulation, in a Gazebo window
nix run github:naval-group/LOTUSim#ui             # the browser interface, on http://localhost:8080
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
