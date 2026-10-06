"""Browser playback for Robotics Toolbox trajectories using mjviser."""

from __future__ import annotations

import os
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING

import mujoco
import numpy as np
import viser
from mjviser import ViserMujocoScene
from numpy.typing import ArrayLike, NDArray

if TYPE_CHECKING:
    from .world import MujocoWorld


@dataclass(frozen=True)
class Motion:
    """A named joint-space motion for browser playback."""

    name: str
    q: NDArray[np.float64]
    t: NDArray[np.float64]
    qd: NDArray[np.float64] | None = None

    @classmethod
    def from_samples(
        cls,
        name: str,
        q: ArrayLike,
        t: ArrayLike | None = None,
        qd: ArrayLike | None = None,
        *,
        default_dt: float = 0.08,
    ) -> Motion:
        positions = np.asarray(q, dtype=float)
        if positions.ndim != 2 or positions.shape[1] != 6 or len(positions) == 0:
            raise ValueError("Motion positions must have shape (samples, 6).")

        if t is None:
            times = np.arange(len(positions), dtype=float) * default_dt
        else:
            times = np.asarray(t, dtype=float)
            if times.shape != (len(positions),):
                raise ValueError("Motion time must have one entry per position sample.")
            times = times - times[0]
            if len(times) > 1 and np.any(np.diff(times) <= 0):
                raise ValueError("Motion time values must be strictly increasing.")

        velocities = None if qd is None else np.asarray(qd, dtype=float)
        if velocities is not None and velocities.shape != positions.shape:
            raise ValueError("Motion velocities must have the same shape as positions.")

        return cls(name, positions.copy(), times.copy(), None if velocities is None else velocities.copy())


def viewer_url(port: int) -> str:
    """Return the user-visible local or GitHub Codespaces viewer URL."""
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


def run_motion_viewer(
    world: MujocoWorld,
    motions: Mapping[str, Motion] | Sequence[Motion],
    *,
    port: int = 8080,
) -> None:
    """Serve an interactive timeline for one or more planned motions."""
    motion_by_name = _normalise_motions(motions)
    motion_names = list(motion_by_name)

    data = mujoco.MjData(world.model)
    server = viser.ViserServer(host="0.0.0.0", port=port)
    print(f"Open the Lab 3 viewer: {viewer_url(port)}")
    print("Press Ctrl+C in this terminal to stop the viewer.")
    scene = ViserMujocoScene(server, world.model, num_envs=1)
    scene.show_contact_points = True
    tabs = scene.create_visualization_gui()

    selected_name = [motion_names[0]]
    frame_index = [0]
    playing = [True]
    looping = [True]
    speed = [1.0]
    elapsed_motion_time = [0.0]
    programmatic_timeline_update = [False]
    needs_render = [True]

    with tabs.add_tab("Planning", icon=viser.Icon.ROUTE):
        motion_selector = server.gui.add_dropdown(
            "Motion",
            options=motion_names,
            initial_value=motion_names[0],
        )
        timeline = server.gui.add_slider(
            "Frame",
            min=0,
            max=len(motion_by_name[motion_names[0]].q) - 1,
            step=1,
            initial_value=0,
        )
        status = server.gui.add_html("")
        play_button = server.gui.add_button("Pause", icon=viser.Icon.PLAYER_PAUSE)
        speed_buttons = server.gui.add_button_group(
            "Speed", options=["0.25x", "0.5x", "1x", "2x", "4x"]
        )
        loop_checkbox = server.gui.add_checkbox("Loop", initial_value=True)

        def set_timeline_value(
            value: int,
            *,
            maximum: int | None = None,
        ) -> None:
            programmatic_timeline_update[0] = True
            try:
                if maximum is not None:
                    timeline.max = maximum
                timeline.value = value
            finally:
                programmatic_timeline_update[0] = False

        @motion_selector.on_update
        def _(_) -> None:
            selected_name[0] = str(motion_selector.value)
            frame_index[0] = 0
            elapsed_motion_time[0] = 0.0
            motion = motion_by_name[selected_name[0]]
            set_timeline_value(0, maximum=len(motion.q) - 1)
            needs_render[0] = True

        @timeline.on_update
        def _(_) -> None:
            if programmatic_timeline_update[0]:
                return
            frame_index[0] = int(timeline.value)
            motion = motion_by_name[selected_name[0]]
            elapsed_motion_time[0] = float(motion.t[frame_index[0]])
            needs_render[0] = True

        @play_button.on_click
        def _(_) -> None:
            playing[0] = not playing[0]
            play_button.label = "Pause" if playing[0] else "Play"
            play_button.icon = (
                viser.Icon.PLAYER_PAUSE if playing[0] else viser.Icon.PLAYER_PLAY
            )

        @speed_buttons.on_click
        def _(event) -> None:
            speed[0] = float(str(event.target.value).removesuffix("x"))

        @loop_checkbox.on_update
        def _(_) -> None:
            looping[0] = bool(loop_checkbox.value)

    def render_frame() -> None:
        motion = motion_by_name[selected_name[0]]
        index = frame_index[0]
        data.qpos[world.qpos_indices] = motion.q[index]
        data.qvel[:] = 0.0
        if motion.qd is not None:
            data.qvel[world.qvel_indices] = motion.qd[index]
        mujoco.mj_forward(world.model, data)
        scene.update_from_mjdata(data)

        contacts: list[str] = []
        clearance_violations: list[str] = []
        for contact in data.contact[: data.ncon]:
            distance = float(contact.dist)
            if distance > world.safety_margin:
                continue
            name1 = world.model.geom(int(contact.geom[0])).name
            name2 = world.model.geom(int(contact.geom[1])).name
            pair = f"{name1} ↔ {name2}"
            if distance <= 0:
                contacts.append(pair)
            else:
                clearance_violations.append(pair)

        if contacts:
            collision_html = (
                '<span style="color:#c0392b"><strong>Collision:</strong> '
                + ", ".join(contacts)
                + "</span>"
            )
        elif clearance_violations:
            collision_html = (
                '<span style="color:#c0392b"><strong>Safety-margin violation:</strong> '
                + ", ".join(clearance_violations)
                + "</span>"
            )
        else:
            collision_html = (
                '<span style="color:#218c4a"><strong>Collision-free</strong> '
                f'(clearance &gt; {world.safety_margin:.3f} m)</span>'
            )

        status.content = (
            f"<div><strong>{motion.name}</strong><br>"
            f"Time: {motion.t[index]:.2f} s &nbsp; "
            f"Frame: {index + 1}/{len(motion.q)}<br>"
            f"{collision_html}</div>"
        )

    render_frame()

    previous_wall_time = time.perf_counter()
    try:
        while True:
            now = time.perf_counter()
            wall_dt = now - previous_wall_time
            previous_wall_time = now

            if playing[0]:
                motion = motion_by_name[selected_name[0]]
                elapsed_motion_time[0] += wall_dt * speed[0]
                end_time = float(motion.t[-1])

                if end_time <= 0:
                    new_index = len(motion.q) - 1
                else:
                    if elapsed_motion_time[0] > end_time:
                        if looping[0]:
                            elapsed_motion_time[0] %= end_time
                        else:
                            elapsed_motion_time[0] = end_time
                            playing[0] = False
                            play_button.label = "Play"
                            play_button.icon = viser.Icon.PLAYER_PLAY
                    new_index = int(
                        np.searchsorted(
                            motion.t,
                            elapsed_motion_time[0],
                            side="right",
                        )
                        - 1
                    )
                    new_index = int(np.clip(new_index, 0, len(motion.q) - 1))

                if new_index != frame_index[0]:
                    frame_index[0] = new_index
                    set_timeline_value(new_index)
                    needs_render[0] = True

            if needs_render[0]:
                render_frame()
                needs_render[0] = False

            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        print("\nViewer stopped.")
    finally:
        server.stop()
