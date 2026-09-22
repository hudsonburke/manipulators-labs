# Robotic Manipulators Labs

## Environment Setup

You can use GitHub Codespaces or a local setup. Both use `uv` for Python dependencies. Codespaces also starts URSim with Docker Compose.

### GitHub Codespaces

The easiest way to get started is to use GitHub Codespaces. The repository's dev container installs `uv`, runs `uv sync`, and starts URSim as a Compose service. Codespaces labels port `6080` as **URSim noVNC** and may open it automatically. If the initial page is a directory listing, append `/vnc.html` to the forwarded URL.

Python commands run through `uv`:

```bash
uv run python lab2/urdf_demo.py
```

To use the RTDE demo, open the **Ports** panel and select the port labeled **URSim noVNC**. Open `/vnc.html` on that forwarded URL, then make sure the simulator is powered on and its safety status is green. From the Codespace terminal, run:

```bash
uv run python lab2/rtde_demo.py
```

The demo reads the joint positions and TCP pose without moving the robot. To run the example joint move explicitly:

```bash
uv run python lab2/rtde_demo.py --move
```

Inside Codespaces, the demo connects to the Compose service named `ursim`.
When running Python directly on a local machine, it defaults to `localhost`.
Override the address with `URSIM_HOST` or `--host` if needed.

#### Creating a Codespace

Open the repository on GitHub, choose **Code → Codespaces → Create codespace on main**, and wait for the container build to finish.

### Local Setup

#### WSL2

If you are on Windows, you'll need to use WSL2 (Windows Subsystem for Linux).
<https://learn.microsoft.com/en-us/windows/wsl/install>

#### git

You'll need git for either branch you decide to use unless you just download the zip file of the repository. You can install git from
<https://git-scm.com/install/>
On Windows, this will also install git bash, which is a terminal that supports unix commands

```bash
git clone <>
```

URSim is available at <http://localhost:6080/vnc.html?host=localhost&port=6080>.

#### Docker

This is what lets you run URSim.
You could also use a full virtual machine, but Docker is easier to set up and use.

- Windows Install: <https://docs.docker.com/desktop/setup/install/windows-install/>
- Mac Install: <https://docs.docker.com/desktop/setup/install/mac-install/>

To run URSim, you can use the following command:

The Compose command above starts URSim and exposes its browser interface on port 6080.

#### `uv`

<https://docs.astral.sh/uv/getting-started/installation/>

```bash
uv sync
```
