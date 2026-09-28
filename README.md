# Robotic Manipulators Labs


## Environment Setup

You can use GitHub Codespaces or a local setup. Both use `uv` for Python dependencies. Codespaces also automatically starts URSim with Docker Compose.

### GitHub Codespaces

The easiest way to get started is to use GitHub Codespaces. 

Sign up for a GitHub account if you don't have one. You can sign up for it with your student email, and you'll get some free perks.

#### Creating a Codespace

![Code button](assets/repo-code-button.png)
Open the repository on GitHub, choose **Code → Codespaces → Create codespace on main**, and wait for the container build to finish.

#### Running URSim

To run a virtual UR5 robot, we are using the URSim Docker image. 
You don't need to worry about the details of this, but think of it like a lightweight virtual machine that runs the same software that runs on a real UR5 robot.
The Docker image is already set up in the Codespace, so you don't need to do anything else.
The Codespace automatically starts URSim with Docker Compose, so you don't need to do anything else.

Programs running inside the Codespace connect to URSim using the Compose
service name `ursim_cb3`. The workspace sets `URSIM_HOST` automatically. For a
local Python process, use `URSIM_HOST=localhost` because Docker publishes the
robot interfaces on the host.

![Ports panel](assets/ports.png)
To open the simulator, open the **Ports** panel and select the port labeled `6080`. 
Open `/vnc.html` on that forwarded URL.

![Port link](assets/port-link.png)

![VNC link](assets/vnc-link.png)

This will open a browser tab

![Confirm safety](assets/confirm-safety.png)

![Go to ](assets/go-to-initialization.png)

![initialization](assets/initialization.png)
At this point, you can hit ok

![](assets/main-menu.png)

![](assets/move-tab.png)

From here you should be able to manually move the robot arm around.

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

[WSL]
  <https://learn.microsoft.com/en-us/windows/wsl/install>
