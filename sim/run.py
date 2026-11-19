"""Fixed-base CubeMars controller experiment. Run with python sim/run.py."""

import argparse
import csv
import time
from contextlib import nullcontext
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import mujoco.viewer
import numpy as np


MODEL_PATH = Path(__file__).resolve().parent / "model" / "robot_legs.xml"
CHANNELS = (
    "hip_pitch", "hip_roll", "hip_yaw", "knee",
    "ankle_outer_crank", "ankle_inner_crank",
)
INNER_KP = np.array([5, 5, 5, 3, 3, 3])
INNER_KD = np.array([0.3, 0.3, 0.3, 0.2, 0.2, 0.2])
OUTER_KP = np.array([5, 5, 5, 25, 25, 25])
OUTER_KI = np.array([0, 0, 0, 0.3, 0.3, 0.3])
OUTER_KD = np.zeros(6)
FIRMWARE_TORQUE_LIMIT = np.array([18, 18, 18, 12, 12, 12])


def load_model(names):
    xml = ET.parse(MODEL_PATH)
    compiler = xml.find("compiler")
    mesh_dir = MODEL_PATH.parent / compiler.get("meshdir")
    compiler.set("meshdir", mesh_dir.as_posix())
    limits = []
    for name, firmware_limit in zip(names, FIRMWARE_TORQUE_LIMIT, strict=True):
        servo = xml.find(f"actuator/position[@name='{name}']")
        low, high = map(float, servo.get("forcerange").split())
        limit = min(-low, high, firmware_limit)
        limits.append(limit)
        joint = servo.get("joint")
        servo.attrib.clear()
        servo.tag = "motor"
        bounds = f"{-limit} {limit}"
        servo.attrib.update(name=name, joint=joint, ctrlrange=bounds, forcerange=bounds)
    return mujoco.MjModel.from_xml_string(ET.tostring(xml.getroot(), encoding="unicode")), np.array(limits)


class Controller:
    def __init__(self, initial, limits, outer_enabled, anti_windup):
        self.command = initial.copy()
        self.previous_position = initial.copy()
        self.velocity = np.zeros_like(initial)
        self.integral = np.zeros_like(initial)
        self.limits = limits
        self.outer_enabled = np.full(initial.shape, outer_enabled)
        self.anti_windup = anti_windup

    def update(self, target, position, velocity, dt):
        remaining = target - self.command
        acceleration = 0.5
        desired = np.sign(remaining) * np.minimum(
            0.3, np.sqrt(2 * acceleration * np.abs(remaining))
        )
        self.velocity += np.clip(
            desired - self.velocity, -acceleration * dt, acceleration * dt
        )
        advance = self.velocity * dt
        reached = (remaining == 0) | (
            (np.sign(advance) == np.sign(remaining))
            & (np.abs(advance) >= np.abs(remaining))
        )
        self.command += np.where(reached, remaining, advance)
        self.velocity[reached] = 0

        error = self.command - position
        derivative = (position - self.previous_position) / dt
        self.previous_position = position.copy()
        self.outer_enabled &= np.abs(error) <= 1.0
        inner = INNER_KP * error + INNER_KD * (self.velocity - velocity)
        proportional_derivative = OUTER_KP * error - OUTER_KD * derivative
        # Store the integral contribution in Nm, including the firmware clamp.
        candidate = np.clip(
            self.integral + OUTER_KI * error * dt, -self.limits, self.limits
        )
        if self.anti_windup:
            outer_request = proportional_derivative + candidate
            total_request = inner + np.clip(outer_request, -self.limits, self.limits)
            blocks_integral = (
                (outer_request * error > self.limits * np.abs(error))
                | (total_request * error > self.limits * np.abs(error))
            )
            candidate = np.where(blocks_integral, self.integral, candidate)
        self.integral = np.where(self.outer_enabled, candidate, 0)
        outer = np.where(
            self.outer_enabled,
            np.clip(proportional_derivative + self.integral, -self.limits, self.limits),
            0,
        )
        return np.clip(inner + outer, -self.limits, self.limits), outer


def experiment(args):
    names = [f"{args.leg}_{channel}" for channel in CHANNELS]
    model, limits = load_model(names)
    data = mujoco.MjData(model)
    motors = np.array([model.actuator(name).id for name in names])
    joints = model.actuator_trnid[motors, 0]
    positions = model.jnt_qposadr[joints]
    velocities = model.jnt_dofadr[joints]
    ankles = [
        model.joint(f"{args.leg}_ankle_{axis}_joint").qposadr[0]
        for axis in ("pitch", "roll")
    ]
    physics_dt = model.opt.timestep
    control_steps = round(args.control_ms / (1000 * physics_dt))
    if control_steps < 1 or not np.isclose(control_steps * physics_dt, args.control_ms / 1000):
        raise ValueError("Control period must be a positive multiple of the physics step.")
    dt = control_steps * physics_dt
    mujoco.mj_forward(model, data)
    # Hold the other leg at its CAD reference with its original position servos.
    other = np.setdiff1d(np.arange(model.nu), motors)
    data.ctrl[other] = data.actuator_length[other]
    initial = data.qpos[positions].copy()
    controller = Controller(initial, limits, args.mode != "mit", args.mode == "anti-windup")
    offset = np.zeros(6)
    if args.motion == "joints":
        offset[:4] = args.amplitude
    else:
        offset[4] = args.amplitude
        offset[5] = args.amplitude if args.motion == "ankle-pitch" else -args.amplitude
    goal = initial + offset
    low, high = model.jnt_range[joints].T
    if np.any((goal < low) | (goal > high)):
        raise ValueError("Requested motion exceeds a simulated joint limit.")

    columns = (
        "target_rad", "command_rad", "position_rad", "velocity_rad_s",
        "torque_nm", "outer_torque_nm",
    )
    peak_error = np.zeros(6)
    saturated = np.zeros(6, dtype=int)
    peak_closure = 0.0
    ankle_start = None
    ticks = 0
    viewer_context = mujoco.viewer.launch_passive(model, data) if args.viewer else nullcontext()
    with args.csv.open("w", newline="", encoding="utf-8") as output, viewer_context as viewer:
        writer = csv.writer(output)
        writer.writerow([
            "time_s", "ankle_pitch_rad", "ankle_roll_rad", "max_rod_closure_error_m",
            *(f"{name}_{column}" for name in names for column in columns),
        ])
        start = time.perf_counter()
        for tick in range(int(np.ceil(args.seconds / dt))):
            if viewer is not None and not viewer.is_running():
                break
            target = initial if data.time < 1.0 else goal
            position = data.qpos[positions]
            velocity = data.qvel[velocities]
            if ankle_start is None and data.time >= 1.0:
                ankle_start = data.qpos[ankles]
            torque, outer = controller.update(target, position, velocity, dt)
            data.ctrl[motors] = torque
            # Forward kinematics and constraint residuals at the logged state.
            mujoco.mj_forward(model, data)
            equality = data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
            closure = np.max(np.abs(data.efc_pos[equality]), initial=0)
            peak_closure = max(peak_closure, closure)
            if not np.isfinite(data.qpos).all() or np.any(data.warning.number):
                raise RuntimeError(f"MuJoCo reported invalid state or warnings at {data.time:.3f} s")
            peak_error = np.maximum(peak_error, np.abs(controller.command - position))
            saturated += np.isclose(np.abs(torque), limits)
            writer.writerow([
                data.time, *data.qpos[ankles], closure,
                *np.column_stack([target, controller.command, position, velocity, torque, outer]).ravel(),
            ])
            mujoco.mj_step(model, data, nstep=control_steps)
            ticks += 1
            if viewer is not None:
                viewer.sync()
                time.sleep(max(0, start + data.time - time.perf_counter()))
    if not np.isfinite(data.qpos).all() or np.any(data.warning.number):
        raise RuntimeError("MuJoCo reported invalid state or warnings on the last step")
    print(f"Simulated {data.time:.3f} s. CSV written to {args.csv.resolve()}")
    print(f"Peak ankle closure residual component: {1000 * peak_closure:.3f} mm")
    if ankle_start is not None:
        pitch, roll = data.qpos[ankles] - ankle_start
        print(f"Ankle change after command: pitch {pitch:+.3f} rad, roll {roll:+.3f} rad")
    for name, peak, count in zip(names, peak_error, saturated, strict=True):
        print(f"{name}: peak tracking error {peak:.4f} rad, torque limited {count}/{ticks} samples")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leg", choices=("left", "right"), default="left")
    parser.add_argument("--motion", choices=("joints", "ankle-pitch", "ankle-roll"), default="joints")
    parser.add_argument("--mode", choices=("firmware", "anti-windup", "mit"), default="firmware")
    parser.add_argument("--amplitude", type=float, default=0.12, help="motor target change in rad")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--control-ms", type=int, choices=range(1, 101), default=5, metavar="1–100")
    parser.add_argument("--csv", type=Path, default=Path("sim_trace.csv"))
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds <= 1 or not np.isfinite(args.amplitude):
        parser.error("Duration must exceed one second and amplitude must be finite.")
    try:
        experiment(args)
    except ValueError as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
