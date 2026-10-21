"""Image-feedback reference reacher for the self-contained rolling-reach scene.

This is a hand-coded baseline, not a Dream policy or a learned-task result.
The policy receives only rendered RGB, measured logical joints, the task text,
and a timestamp. The evaluator keeps marker/tool ground truth separate.
"""

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

try:  # direct script execution from the repository root
    import scene  # noqa: E402
except ImportError:  # package import
    from examples.rolling_reach_arm import scene  # noqa: E402

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"


class ReferenceReacher:
    """RGB-only red-marker reacher with analytic FK/Jacobian and DLS."""

    def __init__(self, cfg=None, calibration=None):
        self.cfg = scene.policy_config(cfg or scene.SCENARIO)
        self.calibration = calibration or scene.camera_calibration(self.cfg)
        self.target = np.asarray(self.cfg.home, dtype=float).copy()
        self.last_goal = None
        self.last_centroid = None
        self.last_detection = None
        self.images_seen = 0
        self.images_with_marker = 0

    def observe(self, rgb, joints, timestamp):
        self.images_seen += 1
        centroid, count, detection = scene.red_center(rgb, self.cfg, self.calibration)
        if centroid is None:
            self.last_centroid = None
            self.last_detection = detection
            return self.target.copy()
        self.images_with_marker += 1
        self.last_centroid = centroid
        self.last_detection = detection
        point, valid = scene.unproject_to_plane(centroid[1], centroid[0],
                                                self.cfg.table_z, self.calibration)
        if not bool(valid):
            return self.target.copy()
        goal = np.array([point[0], point[1], self.cfg.table_z + self.cfg.goal_height])
        if not np.isfinite(goal).all():
            return self.target.copy()
        self.last_goal = goal
        joints = np.asarray(joints, dtype=float)
        position = scene.analytic_fk(joints, self.cfg)
        error = goal - position
        jacobian = scene.analytic_jacobian(joints, self.cfg)
        damping = self.cfg.dls_damping ** 2
        delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * np.eye(3), error)
        largest = float(np.max(np.abs(delta)))
        if largest > self.cfg.max_camera_step:
            delta *= self.cfg.max_camera_step / largest
        proposed = joints + delta
        proposed = self._clip(proposed)
        self.target = proposed
        return self.target.copy()

    def _clip(self, q):
        limits = np.array([self.cfg.yaw_limit,
                           self.cfg.joints[0].bend_limit,
                           self.cfg.joints[1].bend_limit])
        margin = 1e-3
        return np.clip(q, -limits + margin, limits - margin)


class Governor:
    """Acceleration- and speed-limited absolute logical joint target."""

    def __init__(self, command, cfg=None):
        self.cfg = cfg or scene.SCENARIO
        self.command = np.asarray(command, dtype=float).copy()
        self.velocity = np.zeros(3)
        self.max_velocity = 0.
        self.max_acceleration = 0.
        self._last_velocity = self.velocity.copy()

    def step(self, target, dt):
        target = np.asarray(target, dtype=float)
        error = target - self.command
        speed = np.minimum(self.cfg.speed_limit,
                           np.sqrt(2. * self.cfg.accel_limit * np.abs(error)))
        desired = np.sign(error) * speed
        desired[np.abs(error) < 1e-5] = 0.
        dv = np.clip(desired - self.velocity,
                     -self.cfg.accel_limit * dt, self.cfg.accel_limit * dt)
        self.velocity = self.velocity + dv
        self.velocity = np.clip(self.velocity, -self.cfg.speed_limit, self.cfg.speed_limit)
        self.command = self.command + self.velocity * dt
        self.max_velocity = max(self.max_velocity, float(np.max(np.abs(self.velocity))))
        self.max_acceleration = max(self.max_acceleration,
                                    float(np.max(np.abs(self.velocity - self._last_velocity)) / dt))
        self._last_velocity = self.velocity.copy()
        return self.command.copy()


def build_model(cfg=None):
    cfg = cfg or scene.SCENARIO
    return mujoco.MjModel.from_xml_string(scene.generated_xml(cfg))


def set_marker(model, xy, cfg=None):
    cfg = cfg or scene.SCENARIO
    model.geom_pos[model.geom("task_marker").id] = [float(xy[0]), float(xy[1]),
                                                    cfg.marker_half_height]


def marker_goal(model, cfg=None):
    cfg = cfg or scene.SCENARIO
    pos = model.geom_pos[model.geom("task_marker").id]
    return np.array([pos[0], pos[1], cfg.table_z + cfg.goal_height])


def run_episode(cfg=None, case=None, mode="live", shift_xy=None, shift_time=None,
                render_paths=None, seed=0):
    """Run one attempt. mode is live, frozen_image, or hold."""
    cfg = cfg or scene.SCENARIO
    case = case or cfg.probes[0]
    if hasattr(case, "marker_xy"):
        start_xy = np.asarray(case.marker_xy, dtype=float)
    elif hasattr(case, "start_xy"):
        start_xy = np.asarray(case.start_xy, dtype=float)
    else:
        start_xy = np.asarray(case, dtype=float)
    final_xy = np.asarray(shift_xy, dtype=float) if shift_xy is not None else start_xy
    if shift_xy is not None and shift_time is None:
        shift_time = cfg.shift_cases[0].shift_time

    model = build_model(cfg)
    data = mujoco.MjData(model)
    set_marker(model, start_xy, cfg)
    home_internal = scene.logical_to_internal(cfg.home, cfg)
    data.qpos[:] = home_internal
    data.qvel[:] = 0.
    data.ctrl[:] = cfg.home
    mujoco.mj_forward(model, data)

    policy = ReferenceReacher(scene.policy_config(cfg))
    governor = Governor(cfg.home, cfg)
    renderer = mujoco.Renderer(model, height=cfg.image_height, width=cfg.image_width)
    first_image = None

    max_steps = round(cfg.timeout_seconds / cfg.timestep)
    dwell_steps = round(cfg.dwell_seconds / cfg.timestep)
    max_control_speed = 0.
    max_control_accel = 0.
    max_actuator_force = 0.
    max_joint_speed = 0.
    warning_counts = np.zeros_like(data.warning.number)
    contact_count = 0
    current_goal = marker_goal(model, cfg)
    success = False
    success_step = None
    dwell = 0
    min_error = float("inf")
    final_error = float("inf")
    command_before_shift = None
    command_after_shift = None
    samples = []
    frames = []

    def observe(step):
        nonlocal first_image
        renderer.update_scene(data, camera="overhead")
        rgb = renderer.render()
        if mode == "frozen_image":
            if first_image is None:
                first_image = rgb.copy()
            rgb = first_image
        joints = scene.internal_to_logical(data.qpos, cfg)
        return policy.observe(rgb, joints, data.time), rgb

    desired = cfg.home
    for step in range(max_steps):
        if shift_xy is not None and step * cfg.timestep >= shift_time and not np.array_equal(
                model.geom_pos[model.geom("task_marker").id, :2], final_xy):
            set_marker(model, final_xy, cfg)
            mujoco.mj_forward(model, data)
            current_goal = marker_goal(model, cfg)
            if command_before_shift is None:
                command_before_shift = governor.command.copy()

        if step % cfg.camera_decimation == 0:
            if mode == "hold":
                desired = governor.command
                rgb = None
                if render_paths is not None and step == 0:
                    renderer.update_scene(data, camera="overhead")
                    rgb = renderer.render()
            else:
                desired, rgb = observe(step)
            if command_before_shift is not None and command_after_shift is None and np.array_equal(
                    model.geom_pos[model.geom("task_marker").id, :2], final_xy):
                command_after_shift = desired.copy()
            if render_paths is not None and rgb is not None:
                if step % (cfg.camera_decimation * 15) == 0:
                    frames.append(rgb.copy())

        if step % cfg.control_decimation == 0:
            command = governor.step(desired, cfg.timestep * cfg.control_decimation)
            data.ctrl[:] = command
            max_control_speed = max(max_control_speed, governor.max_velocity)
            max_control_accel = max(max_control_accel, governor.max_acceleration)
            tool = data.site_xpos[model.site("tool").id]
            error = float(np.linalg.norm(tool - current_goal))
            min_error = min(min_error, error)
            if step * cfg.timestep >= (shift_time if shift_xy is not None else 0.):
                dwell = dwell + cfg.control_decimation if error < cfg.success_tolerance else 0
                if dwell >= dwell_steps:
                    success = True
                    success_step = step
                    final_error = error
                    if render_paths is not None:
                        samples.append({"t": data.time, "error_m": error,
                                        "command": command.tolist()})
                    break
            if step % (cfg.control_decimation * 10) == 0:
                samples.append({"t": data.time, "error_m": error,
                                "command": command.tolist()})

        mujoco.mj_step(model, data)
        warning_counts += data.warning.number
        contact_count = max(contact_count, int(data.ncon))
        max_actuator_force = max(max_actuator_force, float(np.max(np.abs(data.actuator_force))))
        max_joint_speed = max(max_joint_speed, float(np.max(np.abs(data.qvel))))
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            break

    final_error = float(np.linalg.norm(data.site_xpos[model.site("tool").id] - current_goal))
    renderer.close()

    if render_paths is not None and frames:
        from PIL import Image
        for i, frame in enumerate(frames):
            Image.fromarray(frame).save(render_paths / f"{case_name(case)}_frame_{i:03d}.png")
        if len(frames) > 1:
            images = [Image.fromarray(frame) for frame in frames]
            images[0].save(render_paths / f"{case_name(case)}_motion.gif", save_all=True,
                           append_images=images[1:], duration=100, loop=0)

    return {
        "case": case_name(case),
        "mode": mode,
        "seed": int(seed),
        "marker_start_xy_m": start_xy.tolist(),
        "marker_final_xy_m": final_xy.tolist(),
        "shift_time_s": shift_time,
        "success": bool(success),
        "success_time_s": None if success_step is None else success_step * cfg.timestep,
        "final_error_m": final_error,
        "min_error_m": min_error,
        "dwell_seconds_required": cfg.dwell_seconds,
        "timeout_seconds": cfg.timeout_seconds,
        "simulated_seconds": float(data.time),
        "steps": int(round(data.time / cfg.timestep)),
        "policy_images": policy.images_seen,
        "policy_images_with_marker": policy.images_with_marker,
        "last_goal_estimate": None if policy.last_goal is None else policy.last_goal.tolist(),
        "last_centroid_px": None if policy.last_centroid is None else list(policy.last_centroid),
        "last_detection": policy.last_detection,
        "command_before_shift": None if command_before_shift is None else command_before_shift.tolist(),
        "command_after_shift": None if command_after_shift is None else command_after_shift.tolist(),
        "max_control_speed_rad_s": max_control_speed,
        "max_control_accel_rad_s2": max_control_accel,
        "max_actuator_force_nm": max_actuator_force,
        "max_joint_speed_rad_s": max_joint_speed,
        "max_contacts": contact_count,
        "warning_counts": warning_counts.tolist(),
        "samples": samples,
    }


def case_name(case):
    return getattr(case, "name", "case")


def run_suite(cfg=None, episodes=None, seed=0, render=False):
    cfg = cfg or scene.SCENARIO
    probes = list(cfg.probes)
    if episodes is not None:
        order = np.random.default_rng(seed).permutation(len(probes))
        probes = [probes[i] for i in order[:episodes]]
    render_dir = None
    if render:
        render_dir = RESULTS / "render"
        render_dir.mkdir(parents=True, exist_ok=True)
    attempts = []
    for i, case in enumerate(probes):
        attempt = run_episode(cfg, case, mode="live", seed=seed,
                              render_paths=render_dir if render and i == 0 else None)
        attempts.append(attempt)
        print(f"{attempt['case']}: success={attempt['success']} "
              f"error={attempt['final_error_m']:.4f} m time={attempt['success_time_s']}")
    shifts = []
    for i, case in enumerate(cfg.shift_cases):
        attempt = run_episode(cfg, case, mode="live", shift_xy=case.shift_xy,
                              shift_time=case.shift_time, seed=seed,
                              render_paths=render_dir if render and i == 0 else None)
        shifts.append(attempt)
        print(f"{attempt['case']}: success={attempt['success']} "
              f"error={attempt['final_error_m']:.4f} m time={attempt['success_time_s']}")
    controls = []
    for i, case in enumerate(cfg.shift_cases):
        for mode in ("frozen_image", "hold"):
            attempt = run_episode(cfg, case, mode=mode, shift_xy=case.shift_xy,
                                  shift_time=case.shift_time, seed=seed)
            controls.append(attempt)
            print(f"{attempt['case']} [{mode}]: success={attempt['success']} "
                  f"error={attempt['final_error_m']:.4f} m")
    return {
        "scope": "hand-coded RGB reference baseline; not a Dream policy or learned result",
        "policy_input_contract": "rendered RGB, measured logical joints, fixed task text, "
                                 "timestamp, and fixed camera calibration; no marker or "
                                 "target coordinates",
        "reference_images": True,
        "physics_engine": "MuJoCo",
        "seed": int(seed),
        "attempts": attempts,
        "shift_attempts": shifts,
        "control_attempts": controls,
        "metadata": {
            "cfg": scene.effective_manifest(cfg),
            "versions": _versions(),
        },
    }


def _versions():
    import importlib.metadata
    out = {}
    for name in ("mujoco", "numpy", "Pillow"):
        try:
            out[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            out[name] = None
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=len(scene.SCENARIO.probes))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--output", default=None)
    args = parser.parse_args(argv)
    if args.episodes < 1 or args.episodes > len(scene.SCENARIO.probes):
        parser.error("--episodes must be between 1 and the number of declared probes")
    output = Path(args.output) if args.output else RESULTS / "reach_report.json"
    output = output.resolve()
    if RESULTS.resolve() not in output.parents:
        parser.error("--output must be under examples/rolling_reach_arm/results/")
    RESULTS.mkdir(parents=True, exist_ok=True)
    report = run_suite(episodes=args.episodes, seed=args.seed, render=args.render)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"output": str(output), "attempts": len(report["attempts"]),
                      "shift_attempts": len(report["shift_attempts"]),
                      "control_attempts": len(report["control_attempts"])}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
