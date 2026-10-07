"""Browser playback of recorded joint positions in Swift."""

from __future__ import annotations

import asyncio
import html
import http.server
import json
import os
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from queue import Queue
from threading import Thread
from typing import TYPE_CHECKING
from urllib.parse import unquote, urlsplit

import numpy as np
import swift
import websockets
from numpy.typing import ArrayLike, NDArray
from swift.SwiftRoute import SwiftSocket

if TYPE_CHECKING:
    from .world import CollisionWorld


@dataclass(frozen=True)
class Motion:
    """A named joint-space motion for browser playback."""

    name: str
    q: NDArray[np.float64]
    t: NDArray[np.float64]

    @classmethod
    def from_samples(
        cls,
        name: str,
        q: ArrayLike,
        t: ArrayLike | None = None,
        *,
        default_dt: float = 0.08,
    ) -> Motion:
        positions = np.asarray(q, dtype=float)
        if positions.ndim != 2 or positions.shape[1] == 0 or len(positions) == 0:
            raise ValueError("Motion positions must have shape (samples, joints).")

        if t is None:
            times = np.arange(len(positions), dtype=float) * default_dt
        else:
            times = np.asarray(t, dtype=float)
            if times.shape != (len(positions),):
                raise ValueError("Motion time must have one entry per position sample.")
            times = times - times[0]
            if len(times) > 1 and np.any(np.diff(times) <= 0):
                raise ValueError("Motion time values must be strictly increasing.")

        return cls(name, positions.copy(), times.copy())


def viewer_url(port: int) -> str:
    """Return the user-visible local or private GitHub Codespaces URL."""
    codespace = os.environ.get("CODESPACE_NAME")
    domain = os.environ.get("GITHUB_CODESPACES_PORT_FORWARDING_DOMAIN")
    if codespace and domain:
        return f"https://{codespace}-{port}.{domain}"
    return f"http://localhost:{port}"


def _normalise_motions(
    motions: Mapping[str, Motion] | Sequence[Motion],
) -> dict[str, Motion]:
    if isinstance(motions, Mapping):
        normalised = dict(motions)
        if any(name != motion.name for name, motion in normalised.items()):
            raise ValueError("Motion mapping keys must match Motion.name values.")
    else:
        normalised = {motion.name: motion for motion in motions}
    if not normalised:
        raise ValueError("At least one motion is required for playback.")
    return normalised


class _PlaybackSocket(SwiftSocket):
    """Swift's wire protocol with a caller-selected, exact listening port."""

    def __init__(self, env: swift.Swift, port: int) -> None:
        # Swift 2.0's launch kwargs do not configure ports or remote URLs.
        # Reuse its protocol, but own the transport lifecycle locally.
        self.run = env._servers_running
        self.outq = env.outq
        self.inq = env.inq
        self.disconnected = env._disconnected
        self.USERS = set()
        self.loop = asyncio.new_event_loop()
        self.started = Queue()
        self.thread = Thread(target=self._run, args=(port,), daemon=True)

    async def _listen(self, port: int) -> None:
        self._server = await websockets.serve(self.serve, "0.0.0.0", port)

    def _run(self, port: int) -> None:
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._listen(port))
        except Exception as exc:
            self.started.put(exc)
            self.loop.close()
            raise
        self.started.put(None)
        try:
            self.loop.run_forever()
        finally:
            self._server.close()
            self.loop.run_until_complete(self._server.wait_closed())
            pending = asyncio.all_tasks(self.loop)
            for task in pending:
                task.cancel()
            self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            self.loop.close()


class _PlaybackHTTP(http.server.ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def stop(self) -> None:
        self.shutdown()
        self.server_close()


def _launch_swift(env: swift.Swift, world: CollisionWorld, port: int, websocket_port: int) -> None:
    """Attach Swift to transport supporting private HTTPS/WSS forwarding.

    Swift 2.0 hardcodes localhost in both its server and frontend. Serve its
    assets unchanged except for the WebSocket endpoint and slider update echo:
    frontend programmatic updates must not masquerade as user scrubbing.
    """
    root = Path(swift.__file__).parent / "public"
    socket_url = viewer_url(websocket_port).replace("https://", "wss://").replace("http://", "ws://") + "/"
    main = (root / "js" / "main.js").read_text()
    endpoint = "new WebSocketTransport(`ws://localhost:${portFromLocation()}/`)"
    if endpoint not in main:
        raise RuntimeError("Installed Swift frontend transport changed; update the Lab 3 adapter.")
    main = main.replace(endpoint, f"new WebSocketTransport({json.dumps(socket_url)})").encode()
    ui = (root / "js" / "ui.js").read_text()
    echo = "    this.onInput();"
    if echo not in ui:
        raise RuntimeError("Installed Swift slider update changed; update the Lab 3 adapter.")
    # Programmatic updates must not become user input, but must also preserve
    # a real scrub waiting for the next shape_poses response.
    ui = ui.replace(
        echo,
        "    const pending = this.changed;\n"
        "    const pendingData = this.data;\n"
        "    this.onInput();\n"
        "    this.changed = pending;\n"
        "    if (pending) this.data = pendingData;",
    ).encode()
    shapes = (root / "js" / "shapes.js").read_text()
    mesh_url = (
        '  let filename = part.filename;\n'
        '  if (navigator.appVersion.indexOf("Win") !== -1) {\n'
        '    filename = filename.slice(2);\n'
        '  }\n'
        '  const url = "/retrieve" + encodeURI(filename);'
    )
    if mesh_url not in shapes:
        raise RuntimeError("Installed Swift mesh loading changed; update the Lab 3 adapter.")
    # Mesh paths belong to the Python host, not the browser's OS. Encode the
    # entire path as one URL component rather than stripping a drive prefix
    # based on navigator.appVersion (Windows browsers also access Codespaces).
    shapes = shapes.replace(
        mesh_url, '  const url = "/retrieve/" + encodeURIComponent(part.filename);'
    ).encode()
    scripts = {"/js/main.js": main, "/js/ui.js": ui, "/js/shapes.js": shapes}
    mesh_paths = {
        Path(shape.filename).resolve()
        for link in world.robot.links
        for shape in link.geometry
        if hasattr(shape, "filename") and shape.filename
    }

    class Handler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(root), **kwargs)

        def log_message(self, format, *args):
            pass

        def end_headers(self):
            self.send_header("Cache-Control", "no-store")
            super().end_headers()

        def do_GET(self):
            path = urlsplit(self.path).path
            if path in scripts:
                content = scripts[path]
                self.send_response(200)
                self.send_header("Content-Type", "text/javascript")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            elif path.startswith("/retrieve/"):
                mesh = Path(unquote(path[len("/retrieve/"):])).resolve()
                if mesh not in mesh_paths:
                    self.send_error(404)
                    return
                try:
                    content = mesh.read_bytes()
                except OSError:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", self.guess_type(str(mesh)))
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            else:
                self.path = "/index.html" if path == "/" else path
                super().do_GET()

    # Start in headless mode solely to initialise Swift without its hardcoded
    # transport and blocking built-in pause controls. This is a graphical session.
    env.launch(headless=True, realtime=False)
    env.server = _PlaybackHTTP(("0.0.0.0", port), Handler)
    env.socket = _PlaybackSocket(env, websocket_port)
    env.socket_thread = env.socket.thread
    env.server_thread = Thread(target=env.server.serve_forever, daemon=True)
    env._run_thread = True
    try:
        env.server_thread.start()
        env.socket_thread.start()
        error = env.socket.started.get()
        if error is not None:
            raise error
    except BaseException:
        env._run_thread = False
        if env.socket_thread.is_alive():
            env.socket.stop()
            env.socket_thread.join(1)
        env.server.stop()
        env.server_thread.join(1)
        raise
    env.headless = False
    print(f"Open the Lab 3 viewer: {viewer_url(port)}/", flush=True)
    if socket_url.startswith("wss://"):
        print(f"Keep both ports private. First authenticate the WebSocket port in a browser: {viewer_url(websocket_port)}/", flush=True)
        print("Then open the viewer URL above (the WebSocket port is not a web page).", flush=True)
    print("Waiting for the browser. Press Ctrl+C to stop the viewer.", flush=True)
    # The browser sends Swift's version handshake before any protocol replies.
    env.inq.get()
    env._send_socket("browser_timeout", None, expected=False)


def run_motion_viewer(
    world: CollisionWorld,
    motions: Mapping[str, Motion] | Sequence[Motion],
    *,
    port: int = 52000,
    websocket_port: int = 53000,
) -> None:
    """Play recorded positions and hold at the trajectory's final sample."""
    motion_by_name = _normalise_motions(motions)
    motion_names = list(motion_by_name)
    motion = motion_by_name[motion_names[0]]
    frame = 0
    playing = True
    speed = 1.0
    elapsed = 0.0
    dirty = True
    previous_wall_time = time.perf_counter()
    env = swift.Swift()
    world.robot.control_mode = "p"

    def select_motion(index):
        nonlocal motion, frame, elapsed, dirty, previous_wall_time, playing
        motion = motion_by_name[motion_names[int(index)]]
        frame = 0
        playing = True
        play_button.label = "Pause"
        elapsed = 0.0
        timeline.max = len(motion.q) - 1
        timeline.value = 0
        dirty = True
        previous_wall_time = time.perf_counter()

    def scrub(value):
        nonlocal frame, elapsed, dirty, previous_wall_time
        frame = int(np.clip(int(value), 0, len(motion.q) - 1))
        timeline.value = frame
        elapsed = float(motion.t[frame])
        dirty = True
        previous_wall_time = time.perf_counter()

    def toggle_play(_):
        nonlocal playing, previous_wall_time
        playing = not playing
        play_button.label = "Pause" if playing else "Play"
        previous_wall_time = time.perf_counter()

    def select_speed(index):
        nonlocal speed, previous_wall_time
        speed = (0.25, 0.5, 1.0, 2.0, 4.0)[int(index)]
        previous_wall_time = time.perf_counter()

    timeline = swift.Slider(scrub, min=0, max=len(motion.q) - 1, step=1, value=0, label="Frame", precision=0)
    play_button = swift.Button(toggle_play, label="Pause")
    status = swift.Label("")
    try:
        env.headless = True
        _launch_swift(env, world, port, websocket_port)
        world.check_configuration(motion.q[0])
        robot_view = env.add_assembly(
            world.robot.fkine_geometry,
            [shape for link in world.robot.links for shape in link.geometry],
            q0=motion.q[0],
            readonly=True,
            name=world.robot.name,
        )
        robot_view.control_mode = "p"
        for name, obstacle in world.obstacles.items():
            env.add_shape(obstacle, name=name)
        for element in (
            swift.Select(select_motion, label="Motion", options=motion_names),
            timeline,
            play_button,
            swift.Select(select_speed, label="Speed", options=["0.25x", "0.5x", "1x", "2x", "4x"], value=2),
            status,
        ):
            env.add_ui(element)
        previous_wall_time = time.perf_counter()
        while True:
            now = time.perf_counter()
            wall_dt = now - previous_wall_time
            previous_wall_time = now
            if playing:
                elapsed += wall_dt * speed
                end_time = float(motion.t[-1])
                if elapsed >= end_time:
                    elapsed = end_time
                    playing = False
                    play_button.label = "Play"
                new_frame = int(np.clip(np.searchsorted(motion.t, elapsed, side="right") - 1, 0, len(motion.q) - 1))
                if new_frame != frame:
                    frame = new_frame
                    timeline.value = frame
                    dirty = True
            if dirty:
                report = world.check_configuration(motion.q[frame])
                robot_view.q = motion.q[frame]
                status.label = (
                    f"{html.escape(motion.name)}<br>Time: {motion.t[frame]:.2f} s; "
                    f"Frame: {frame + 1}/{len(motion.q)}<br>"
                    f"{html.escape(report.summary())}<br>"
                    f"Required clearance: {world.safety_margin:.3f} m"
                )
                dirty = False
            # Always poll UI events, even while paused. Position control and
            # zero simulation dt keep playback fixed at the recorded pose.
            env.step(0.0)
            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        print("\nViewer stopped.")
    finally:
        if not env.headless and env._run_thread:
            env.close()
