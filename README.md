![Logo](docs/lotusim_logo.svg)

![Different drones in LOTUSim.](docs/lotusim_environment.png)

LOTUSim is a real-time, multi-domain simulation platform for maritime operations. It models realistic surface, underwater, and air physics for aerial drones, surface ships, and underwater vehicles. An immersive interface lets human operators run human-autonomous agent scenarios, and physically accurate models make LOTUSim suitable for training AI algorithms.

Jump to the [Quickstart](#quickstart) below to get it installed and running in about 10 minutes. For everything else — tutorials, available models/sensors/batteries, the [Developer Workflow](https://github.com/naval-group/LOTUSim/wiki/Developer-Workflow), and architecture — see the [wiki](https://github.com/naval-group/LOTUSim/wiki).

## Quickstart

**Get LOTUSim installed and running a live example in about 10 minutes!**

LOTUSim is installed and run through **Nix**, a package manager that downloads everything the simulator needs (ROS 2, Gazebo, and all their dependencies) into its own isolated location on your computer. It won't conflict with or change anything else you have installed.

You don't need to know Nix to use LOTUSim. Follow the steps below in order and you'll have a working simulation in a few minutes.

LOTUSim runs natively on **Linux and macOS**.

> **On Windows?** Nix doesn't run natively on Windows. Skip ahead to [Windows users](#windows-users). You'll run everything inside WSL2, then the rest of this guide applies exactly as written.

### One-time setup

Do this section once, no matter which path you choose.

#### Step 1 - Install Nix

Open a terminal and run:

```sh
curl --proto '=https' --tlsv1.2 -L https://nixos.org/nix/install | sh -s -- --daemon
```

Once it finishes, **close your terminal window and open a new one** so the changes take effect. Other install options (for special setups) are listed on the [Nix website](https://nixos.org/download/).

#### Step 2 - Add the binary caches

LOTUSim depends on Gazebo/ROS packages, and on Naval Group's own builds, that aren't part of Nix's normal download cache. Without this step, the very first time you build or run LOTUSim, Nix will quietly compile those packages from scratch, that can take **about an hour**, with no progress message explaining why. This step also enables the `nix-command` and `flakes` experimental features that later steps need, and lets Nix download the prebuilt packages instead, which takes seconds.

It needs admin rights, but only once per machine:

```sh
sudo tee -a /etc/nix/nix.conf <<'EOF'
experimental-features = nix-command flakes
extra-substituters = https://ros.cachix.org https://naval-group.cachix.org
extra-trusted-public-keys = ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo= naval-group.cachix.org-1:ytTEzFEeuzQrC9IRYLzHGa5OnM65G95M6/sbPd0fy28=
EOF
sudo systemctl restart nix-daemon   # macOS: sudo launchctl kickstart -k system/org.nixos.nix-daemon
```

> Without a trusted key the cache's signatures do not verify, so Nix builds from source anyway. That is why this step is not optional. A single-user install has no daemon and no `trusted-users`, and can skip it.
> If `systemctl` isn't found (e.g. macOS or a single-user Nix install), restart whichever service manager your install uses, or simply restart your computer.

`/etc/nix/nix.conf` is the system-wide file, so the keys are trusted for every user and the signatures verify.

#### Step 3 - Install the GPU bridge

Skip this step if you just want to use Podman (Path A, Option 2).

##### What is a GPU bridge?

LOTUSim's 3D window needs to talk to your graphics card. On most Linux distributions, Nix programs can't see your machine's GPU by default. Without this bridge, the simulation window may not open.

Install the one matching your graphics card:
```sh
nix profile add github:nix-community/nixGL#nixGLIntel
```

On an NVIDIA or hybrid/Optimus machine, also add the NVIDIA bridge so rendering uses the discrete GPU instead of falling back to Intel — this reads your driver's exact version off the running machine, so it needs `--impure`, and NVIDIA's userspace driver is unfree:

```sh
NIXPKGS_ALLOW_UNFREE=1 nix profile add --impure github:nix-community/nixGL#nixGLNvidia
```

You're now ready to run LOTUSim!

### Pick your path

There are three ways to get LOTUSim, depending on what you want to do:

| I want to... | Use this path |
|---|---|
| Just try it out, nothing left behind afterwards | [Path A - Run without installing](#path-a---run-without-installing) |
| Have `lotusim` available any time, like a normal app | [Path B - Install LOTUSim](#path-b---install-lotusim) |
| Change or contribute to LOTUSim's code | [Path C - Set up your dev environment](#path-c---set-up-your-dev-environment) |

### Path A - Run without installing

Good for a first try. Nothing is added to your system permanently (aside from Nix itself).

Pick **one** of these two:

**Option 1 - via Nix**
```sh
nix run github:naval-group/LOTUSim -- run --gui
```

**Option 2 - via a container (Podman)**

You'll need **Podman version 6 or later** installed, check with `podman --version`, and upgrade if it's older. Then:

```sh
podman run --rm ghcr.io/naval-group/lotusim
```

For **Nvidia** cards, you will need the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) installed and configured to use Podman, or the 3D window won't render.

Once it's running, jump to [Try your first scenario](#try-your-first-scenario).

### Path B - Install LOTUSim

This puts `lotusim` and `lotusim-ui` permanently on your computer, like installing a normal application.

#### 1. Install it
```sh
nix profile add github:naval-group/LOTUSim github:naval-group/LOTUSim#ui
```

#### 2. Run it
```sh
lotusim run --gui
```

In a second terminal:
```sh
lotusim-ui
```

Then open **http://localhost:8080** in your browser.

`lotusim --help` lists the worlds a build carries.

Now head to [Try your first scenario](#try-your-first-scenario).

### Path C - Set up your dev environment

If you're going to modify LOTUSim's code, you want the developer workflow:

```sh
git clone https://github.com/naval-group/LOTUSim.git
cd LOTUSim
nix develop             # drops you into a shell with ROS 2, Gazebo, and build tools
mise run build          # builds the workspace
mise run sim            # runs it. You can add the param --gui
```

This gets the core simulator running. If you also want to work on the web UI or the physics engine (xdyn), or you want the full task reference, see [Developing for LOTUSim](https://github.com/naval-group/LOTUSim/wiki/Getting-Started#developing-for-lotusim) on the wiki.

### Windows users

Nix isn't natively supported on Windows, so you'll run it inside **WSL2** (Windows Subsystem for Linux). It's a lightweight Linux environment that runs alongside Windows. Once it's set up, everything else in this guide (Paths A, B, and C) works exactly as written, from inside your WSL2 terminal.

1. Open PowerShell **as Administrator** and run:
```powershell
wsl --install -d Ubuntu-24.04
```
2. Enable mirrored networking, so ROS 2's device discovery works without extra configuration. Open:
```powershell
notepad $env:USERPROFILE\.wslconfig
```
   and add:
```ini
[wsl2]
networkingMode=mirrored
```
3. Restart WSL, then open your Ubuntu shell and follow **any of the paths above** (A, B, or C) exactly as written.

> If you hit networking issues after this, check your Windows Firewall, mirrored networking is sometimes blocked by default.

### Try your first scenario

1. With LOTUSim and the web UI both running, open the UI in your browser:
   - Installed via Path B: **http://localhost:8080**
   - Dev setup (Path C, `nix run .#ui`): **http://localhost:8080**; if running the frontend directly with `npm run dev`, it's **http://localhost:5173**
2. In the left panel, under **"Launch Scenario"**, select **`demo.yaml`**.
3. Click **Launch Scenario**.

You should see an arrow representing an LRAUV (an underwater vehicle) appear and start moving. 🎉

Want to see more? Check the [Tutorial page](https://github.com/naval-group/LOTUSim/wiki/Tutorial) for further examples, or run `lotusim --help` to see every scenario/world your build includes.

### Optional: Set up the 3D rendering

> This is only needed if you want photorealistic rendering through Unity. The demo scenario above works without it.

1. **Install Unity**
   - Download the [Unity Hub](https://unity.com/download) and follow the on-screen installation guide.
   - Create or sign in to your Unity account (UDN), choose a license type, and then install the Unity Editor through Unity Hub.
   - Use Unity version **_2022.3.18f1_** (required for HDRP water system).

2. **Clone the Unity project**
```bash
git clone --recurse-submodules https://github.com/naval-group/LOTUSim-Unity-modules
cd LOTUSim-Unity-modules
git submodule update --remote --merge
```

3. **Open the project**
   - In Unity Hub -> **Projects**, add the "LOTUSim-Unity-modules" folder and open it.
   - Once the project opens, go to the **Project** tab (bottom panel), open the **Scenes** folder, and load one of the scenes.

4. **Launch a scene**

In Unity Hub, open a scene from the imported project (for example, the defense scenario).

> If the scene appears black, check that your graphics drivers are properly installed.

The [wiki](https://github.com/naval-group/LOTUSim/wiki) covers upgrading an install, composing extra asset roots, the state directory, and developing the web UI or physics engine.

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
