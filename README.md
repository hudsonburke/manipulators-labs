# Robotic Manipulators Labs

## Environment Setup

You can use GitHub Codespaces or a local setup. Both use `uv` for Python dependencies. Codespaces also starts URSim with Docker Compose.

### GitHub Codespaces

The easiest way to get started is to use GitHub Codespaces. The repository's dev container installs `uv`, runs `uv sync`, and starts URSim as a Compose service. Codespaces labels port `6080` as **URSim noVNC** and may open it automatically. If the initial page is a directory listing, append `/vnc.html` to the forwarded URL.

Python commands run through `uv`:

```bash
uv run python lab2/urdf_demo.py
```

This demo uses the Robotics Toolbox **Swift** browser visualizer, not URSim.
The script starts Swift on ports `52000` (web) and `53000` (websocket). Open
the forwarded port labeled **Swift visualizer**, then add `/?53000` to its
URL. Leave the script running while the visualizer is open.

![Ports panel](ports-panel.png)
To use the RTDE demo, open the **Ports** panel and select the port labeled `6080`. Open `/vnc.html` on that forwarded URL, then make sure the simulator is powered on and its safety status is green.
![Image of the hyperlinks shown when clicking on that link](vnc-link.png)

[](https://sdurobotics.gitlab.io/ur_rtde/pages/getting_started/environment_setup.html#simulator-setup:~:text=The%20following%20images%20show%20the%20typical%20sequence%20for%20confirming%20the%20safety%20configuration%20and%20starting%20the%20robot%2E%20In%20each%20image%2C%20the%20action%20the%20user%20must%20perform%20is%20highlighted%20with%20a%20red%20box)

- Press `Confirm Saftey Configuration`
- In the bottom left corner, click the `Power off` button
- Click `On`
- Click `Start`

From the Codespace terminal, run:

```bash
uv run python lab2/rtde_test.py
```

Inside Codespaces, the demo connects to the Compose service named `ursim`.
When running Python directly on a local machine, it defaults to `localhost`.
Override the address with `URSIM_HOST` if needed.

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
