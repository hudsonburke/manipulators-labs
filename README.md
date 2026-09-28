# Robotic Manipulators Labs


## Environment Setup

You can use GitHub Codespaces or a local setup. Both use `uv` for Python dependencies. Codespaces also automatically starts URSim with Docker Compose.

### GitHub Codespaces

The easiest way to get started is to use GitHub Codespaces. 

Sign up for a GitHub account if you don't have one. You can sign up for it with your student email, and you'll get some free perks.

#### Creating a Codespace


![Code button](lab2/assets/repo-code-button.png)
Open the repository on GitHub, choose **Code → Codespaces → Create codespace on main**, and wait for the container build to finish.

The repository's dev container installs `uv`, runs `uv sync`, and starts URSim as a Compose service. Codespaces labels port `6080` as **URSim noVNC** and may open it automatically. If the initial page is a directory listing, append `/vnc.html` to the forwarded URL.

To use the RTDE demo, open the **Ports** panel and select the port labeled `6080`. Open `/vnc.html` on that forwarded URL, then make sure the simulator is powered on and its safety status is green.

[URSim-setup]

- Press `Confirm Saftey Configuration`
- In the bottom left corner, click the `Power off` button
- Click `On`
- Click `Start`

### Local Setup

#### WSL2

If you are on Windows, you'll need to use WSL2 (Windows Subsystem for Linux).
[WSL]

#### git

You'll need git for either branch you decide to use unless you just download the zip file of the repository. You can install git from
<https://git-scm.com/install/>
On Windows, this will also install git bash, which is a terminal that supports unix commands

```sh
git clone https://github.com/hudsonburke/manipulators-labs.git
```

URSim is available at <http://localhost:6080/vnc.html?host=localhost&port=6080>.

#### Docker

This is what lets you run URSim.
You could also use a full virtual machine, but Docker is easier to set up and use.

- Windows Install: <https://docs.docker.com/desktop/setup/install/windows-install/>
- Mac Install: <https://docs.docker.com/desktop/setup/install/mac-install/>
- Linux Install: If you are on Linux, I assume that you can figure this out for your distro

To run URSim, you can use the following command:

The Compose command above starts URSim and exposes its browser interface on port 6080.

#### `uv`

<https://docs.astral.sh/uv/getting-started/installation/>

```sh
uv sync
```

[URSim-setup]: 
  <https://sdurobotics.gitlab.io/ur_rtde/pages/getting_started/environment_setup.html#simulator-setup>
  "URSim setup instructions"
[WSL]
  <https://learn.microsoft.com/en-us/windows/wsl/install>
