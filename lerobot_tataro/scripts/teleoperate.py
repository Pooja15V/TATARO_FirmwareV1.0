#!/usr/bin/env python
"""Teleoperate the TATARO Lite6 with an Android phone (WebXR). Nothing is recorded.

    conda activate lerobot
    python teleoperate.py --no-cameras          # first test: arm only
    python teleoperate.py --display             # with both RealSense cameras in Rerun

Open the https URL printed by the script in Chrome on the phone (same network), accept the
certificate warning, tap Start. Hold the phone screen-up with its top edge pointing away from
the robot base (+X). Touch-and-move once to calibrate, then:
    hold Move  -> arm follows the phone (release = arm holds still)
    A / B      -> gripper open / close
Ctrl+C to stop. KEEP THE E-STOP IN REACH.
"""

import argparse
import time

from tataro_common import add_common_args, make_phone, make_processors, make_robot

from lerobot.utils.rerun_visualization import init_rerun, log_rerun_data


def check_axes():
    """Robot stays disconnected. Hold Move and move the phone; prints what the TCP would do."""
    import json
    from pathlib import Path

    log_path = Path.home() / "lerobot_data" / "phone_axes_log.jsonl"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = log_path.open("w")
    t0 = time.time()
    phone = make_phone()
    orig_cb = phone._on_message

    def logging_cb(pose, message):
        log.write(json.dumps({"t": round(time.time() - t0, 3), **message}, default=str) + "\n")
        log.flush()
        orig_cb(pose, message)

    phone._on_message = logging_cb
    print(f"Raw phone messages are logged to {log_path}")
    phone.connect()
    print("Hold Move and push the phone forward / left / up. Expected: +X / +Y / +Z. Ctrl+C to quit.\n")
    edge = None
    try:
        while True:
            a = phone.get_action()
            if a and a["phone.enabled"]:
                if edge is None:
                    edge = (a["phone.pos"].copy(), a["phone.rot"])
                d = (a["phone.pos"] - edge[0]) * 100
                rot = (a["phone.rot"] * edge[1].inv()).as_rotvec()
                axis = "XYZ"[int(abs(d).argmax())]
                sign = "+" if d[abs(d).argmax()] > 0 else "-"
                print(
                    f"\rrobot dX={d[0]:+6.1f} dY={d[1]:+6.1f} dZ={d[2]:+6.1f} cm  -> mostly {sign}{axis}   "
                    f"rot(deg) rx={rot[0] * 57.3:+5.0f} ry={rot[1] * 57.3:+5.0f} rz={rot[2] * 57.3:+5.0f}  "
                    f"grip={a['phone.gripper']}   ",
                    end="",
                    flush=True,
                )
            else:
                edge = None
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        phone.disconnect()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p)
    p.add_argument("--home-on-exit", action="store_true")
    p.add_argument("--check-axes", action="store_true", help="phone only, robot NOT connected: print mapped motion")
    args = p.parse_args()

    if args.check_axes:
        check_axes()
        return

    robot = make_robot(args)
    phone = make_phone()
    teleop_proc, robot_proc, obs_proc = make_processors(args)

    print("Connecting Lite6 (the arm will move to HOME) ...")
    robot.connect()
    print("Starting phone server ...")
    phone.connect()  # blocks until the phone calibrates
    if args.display:
        init_rerun(session_name="tataro_lite6_teleop")

    dt = 1.0 / args.fps
    last_print = 0.0
    try:
        while True:
            t0 = time.perf_counter()
            obs = obs_proc(robot.get_observation())
            act = phone.get_action()
            if act:
                ee_act = teleop_proc((act, obs))
                sent = robot.send_action(robot_proc((ee_act, obs)))
                if args.display:
                    log_rerun_data(observation=obs, action=sent)
                if t0 - last_print > 0.5:
                    print(
                        f"\rTCP x={obs['ee.x']:.3f} y={obs['ee.y']:.3f} z={obs['ee.z']:.3f} m  "
                        f"grip={'CLOSED' if sent['gripper.pos'] else 'open  '}",
                        end="",
                        flush=True,
                    )
                    last_print = t0
            time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        if args.home_on_exit and robot.is_connected:
            robot.go_home()
        robot.disconnect()
        phone.disconnect()


if __name__ == "__main__":
    main()
