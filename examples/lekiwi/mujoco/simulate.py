#!/usr/bin/env python
"""Interactive MuJoCo simulation of the LeKiwi mobile manipulator.

Run:
    python examples/lekiwi/mujoco/simulate.py

Controls (the MuJoCo window must have focus):
    W / S        drive forward / backward (hold)
    A / D        drive left / right (hold)
    Z / X        rotate left / right (hold)
    1 .. 6       select an arm joint
    Up / Down    move the selected joint (hold to keep moving)
    R            reset base to origin and arm to home pose
    Q / Esc      quit

The arm tracks its joint targets through position servos; the base is driven
through velocity servos, matching the real LeKiwi action interface
(`arm_*.pos` + `x.vel / y.vel / theta.vel`).

Implementation note: the MuJoCo viewer has its own default keyboard shortcuts
(e.g. `W` toggles wireframe), so we pass a `key_callback` to `launch_passive`
to override them. Discrete keys (joint select / step / reset / quit) are
handled there because it reliably receives arrow keys and Escape; continuous
driving (hold W/A/S/D/Z/X) is handled by `pynput`, which provides key
press/release state.
"""

import queue
import time
from pathlib import Path

import glfw
import mujoco
import mujoco.viewer
import numpy as np
from pynput import keyboard

MODEL_PATH = str(Path(__file__).parent / "lekiwi.xml")

ARM_ACTUATORS = [
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
]
ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Base velocity commands (m/s and rad/s), matching the LeKiwi teleop speeds.
BASE_LIN_SPEED = 0.4
BASE_ROT_SPEED = 1.0

# Arm joint step (radians) per Up/Down key press.
ARM_STEP = 0.05


def main() -> None:
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    arm_act_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ARM_ACTUATORS]
    arm_jnt_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ARM_JOINTS]
    arm_limits = model.jnt_range[arm_jnt_ids]
    base_act_ids = {
        n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        for n in ("base_x", "base_y", "base_yaw")
    }

    arm_target = np.zeros(len(ARM_JOINTS))  # arm joint targets (radians)
    selected_joint = 0
    running = True
    pressed: set[str] = set()  # held drive keys (w/a/s/d/z/x)
    actions: queue.Queue = queue.Queue()  # discrete key events, drained in the loop

    def key_callback(key_code: int) -> None:
        """MuJoCo viewer key callback (overrides the viewer's default shortcuts)."""
        if glfw.KEY_1 <= key_code <= glfw.KEY_6:
            actions.put(("select", key_code - glfw.KEY_1))
        elif key_code == glfw.KEY_UP:
            actions.put(("step", +1))
        elif key_code == glfw.KEY_DOWN:
            actions.put(("step", -1))
        elif key_code == glfw.KEY_R:
            actions.put(("reset", None))
        elif key_code in (glfw.KEY_Q, glfw.KEY_ESCAPE):
            actions.put(("quit", None))

    def on_press(key):
        if isinstance(key, keyboard.KeyCode) and key.char and key.char.lower() in "wasdzx":
            pressed.add(key.char.lower())

    def on_release(key):
        if isinstance(key, keyboard.KeyCode) and key.char and key.char.lower() in "wasdzx":
            pressed.discard(key.char.lower())

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    def drain_actions():
        nonlocal selected_joint, running
        while not actions.empty():
            action, value = actions.get_nowait()
            if action == "select":
                selected_joint = value
                lo, hi = arm_limits[selected_joint]
                print(
                    f"[arm] selected: {ARM_JOINTS[selected_joint]} "
                    f"(target={arm_target[selected_joint]:+.3f} rad, limits=[{lo:+.3f}, {hi:+.3f}])"
                )
            elif action == "step":
                arm_target[selected_joint] += ARM_STEP * value
                np.clip(arm_target, arm_limits[:, 0], arm_limits[:, 1], out=arm_target)
                print(
                    f"[arm] {ARM_JOINTS[selected_joint]} target = {arm_target[selected_joint]:+.3f} rad"
                )
            elif action == "reset":
                mujoco.mj_resetData(model, data)
                arm_target[:] = 0.0
                print("[reset] base and arm reset to home")
            elif action == "quit":
                running = False

    def apply_control():
        # Base: hold-to-drive via pynput.
        vx = ("w" in pressed) - ("s" in pressed)
        vy = ("a" in pressed) - ("d" in pressed)
        vyaw = ("z" in pressed) - ("x" in pressed)
        data.ctrl[base_act_ids["base_x"]] = BASE_LIN_SPEED * vx
        data.ctrl[base_act_ids["base_y"]] = BASE_LIN_SPEED * vy
        data.ctrl[base_act_ids["base_yaw"]] = BASE_ROT_SPEED * vyaw
        data.ctrl[arm_act_ids] = arm_target

    print(__doc__)
    print("Arm joints: 1=shoulder_pan 2=shoulder_lift 3=elbow_flex 4=wrist_flex 5=wrist_roll 6=gripper\n")

    t0 = time.time()
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running() and running:
            drain_actions()
            # Step the simulation to keep up with wall-clock (real-time).
            while data.time < time.time() - t0:
                apply_control()
                mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.005)

    listener.stop()


if __name__ == "__main__":
    main()
