"""Read URSim state and optionally move it to a joint-space home pose."""

import argparse
import os

from rtde_receive import RTDEReceiveInterface as RTDEReceive


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--host",
        default=os.getenv("URSIM_HOST", "localhost"),
        help="robot/URSim hostname or IP (default: URSIM_HOST or localhost)",
    )
    parser.add_argument(
        "--move",
        action="store_true",
        help="move URSim to the example home pose after reading its state",
    )
    args = parser.parse_args()

    print(f"Connecting to URSim at {args.host}...")
    receiver = RTDEReceive(args.host)
    try:
        joint_positions = receiver.getActualQ()
        tcp_pose = receiver.getActualTCPPose()

        print("Joint positions [rad]:")
        for index, angle in enumerate(joint_positions, start=1):
            print(f"  q{index}: {angle:.6f}")
        print(f"TCP position [m]: {tcp_pose[:3]}")
        print(f"TCP axis-angle [rad]: {tcp_pose[3:]}")
    finally:
        receiver.disconnect()

    if not args.move:
        print("Read-only demo complete. Use --move to command URSim.")
        return

    from rtde_control import RTDEControlInterface as RTDEControl

    home_q = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    controller = RTDEControl(args.host)
    try:
        print("Moving URSim to the example home pose...")
        if not controller.moveJ(home_q, 0.5, 0.5):
            raise RuntimeError("URSim rejected the moveJ command")
    finally:
        controller.stopScript()
        controller.disconnect()

    print("Move complete.")


if __name__ == "__main__":
    main()
