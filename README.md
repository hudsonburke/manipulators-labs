# Robotic Manipulators Labs

## Environment Setup

You can use GitHub Codespaces or a local setup. Both use `uv` for Python dependencies. Codespaces also starts URSim with Docker Compose.

### GitHub Codespaces

The easiest way to get started is to use GitHub Codespaces. The repository's dev container installs `uv`, runs `uv sync`, and starts URSim as a Compose service. After the Codespace starts, open the forwarded port `6080` to use the URSim browser interface.

Python commands run through `uv`:

```bash
uv run python lab2/urdf_demo.py
```

### Local Setup

#### WSL2

If you are on Windows, you'll need to use WSL2 (Windows Subsystem for Linux).
<https://learn.microsoft.com/en-us/windows/wsl/install>

#### git

You'll need git for either branch you decide to use unless you just download the zip file of the repository. You can install git from
<https://git-scm.com/install/>
On windows, this will also install git bash, which is a terminal that supports unix commands

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
