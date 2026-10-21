"""Independent checks for sim_rolling_reach_v1 and its RGB reference reacher."""

import argparse
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import yourdfpy  # noqa: E402

try:  # direct script execution from the repository root
    import reach  # noqa: E402
except ImportError:  # package import
    from examples.rolling_reach_arm import reach  # noqa: E402
try:  # direct script execution from the repository root
    import scene  # noqa: E402
except ImportError:  # package import
    from examples.rolling_reach_arm import scene  # noqa: E402

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
CFG = scene.SCENARIO


def close(a, b, tolerance, label):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    error = float(np.max(np.abs(a - b)))
    if not error <= tolerance:
        raise AssertionError(f"{label}: max error {error} > {tolerance}")
    return error


def _versions():
    out = {}
    for name in ("mujoco", "numpy", "yourdfpy", "Pillow"):
        try:
            out[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            out[name] = None
    return out


def freeze_cases(cfg=CFG):
    cases = []
    for case in cfg.probes:
        workspace = scene.marker_in_workspace(case.marker_xy, cfg)
        residual = reachability_error(case.marker_xy, cfg)
        cases.append({"kind": "probe", "name": case.name,
                      "marker_xy_m": list(case.marker_xy),
                      "in_declared_workspace": workspace,
                      "ik_reach_residual_m": residual,
                      "excluded": (not workspace) or residual > 1e-3})
    for case in cfg.shift_cases:
        start_workspace = scene.marker_in_workspace(case.start_xy, cfg)
        shift_workspace = scene.marker_in_workspace(case.shift_xy, cfg)
        start_residual = reachability_error(case.start_xy, cfg)
        shift_residual = reachability_error(case.shift_xy, cfg)
        cases.append({"kind": "marker_shift", "name": case.name,
                      "start_xy_m": list(case.start_xy), "shift_xy_m": list(case.shift_xy),
                      "shift_time_s": case.shift_time,
                      "start_in_workspace": start_workspace,
                      "shift_in_workspace": shift_workspace,
                      "start_ik_reach_residual_m": start_residual,
                      "shift_ik_reach_residual_m": shift_residual,
                      "excluded": ((not start_workspace) or (not shift_workspace)
                                   or start_residual > 1e-3 or shift_residual > 1e-3)})
    payload = {
        "model": cfg.name,
        "revision": cfg.revision,
        "manifest": scene.effective_manifest(cfg),
        "cases": cases,
        "declared_before_evaluation": True,
        "criterion": {"goal_height_m": cfg.goal_height,
                      "success_tolerance_m": cfg.success_tolerance,
                      "dwell_seconds": cfg.dwell_seconds,
                      "timeout_seconds": cfg.timeout_seconds},
    }
    text = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    (RESULTS / "probes_frozen.json").write_text(text)
    return {"path": "results/probes_frozen.json",
            "sha256": hashlib.sha256(text.encode()).hexdigest(),
            "cases": cases}


def reachability_error(xy, cfg=CFG):
    goal = np.array([xy[0], xy[1], cfg.goal_height])
    best = float("inf")
    seed = np.asarray(cfg.home, dtype=float)
    for yaw in np.linspace(-cfg.yaw_limit, cfg.yaw_limit, 25):
        for shoulder in np.linspace(-cfg.joints[0].bend_limit,
                                    cfg.joints[0].bend_limit, 31):
            for elbow in np.linspace(-cfg.joints[1].bend_limit,
                                     cfg.joints[1].bend_limit, 31):
                candidate = np.array([yaw, shoulder, elbow])
                distance = float(np.linalg.norm(scene.analytic_fk(candidate, cfg) - goal))
                if distance < best:
                    best = distance
                    seed = candidate
    q = seed.copy()
    limits = np.array([cfg.yaw_limit, cfg.joints[0].bend_limit,
                       cfg.joints[1].bend_limit])
    for _ in range(120):
        error = goal - scene.analytic_fk(q, cfg)
        jacobian = scene.analytic_jacobian(q, cfg)
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + 1e-6 * np.eye(3), error)
        largest = float(np.max(np.abs(delta)))
        if largest > .15:
            delta *= .15 / largest
        q = np.clip(q + delta, -limits + 1e-3, limits - 1e-3)
    return float(np.linalg.norm(scene.analytic_fk(q, cfg) - goal))


def check_config():
    from dataclasses import replace

    for cfg in (CFG, replace(CFG, tool_lateral_offset=.045)):
        model = reach.build_model(cfg)
        declared = scene.effective_manifest(cfg)["tool"]["local_offset_m"]
        close(model.body("tool_payload").pos, declared, 1e-12, "manifest tool offset")
        urdf = scene.urdf(cfg)
        exported = urdf.find("./joint[@name='tool_fixed']/origin").get("xyz")
        close(np.fromstring(exported, sep=" "), declared, 1e-12, "URDF tool offset")
        data = mujoco.MjData(model)
        data.qpos[:] = scene.logical_to_internal(cfg.home, cfg)
        mujoco.mj_forward(model, data)
        close(data.site_xpos[model.site("tool").id], scene.analytic_fk(cfg.home, cfg),
              1e-10, "configured tool FK")
    return scene.validate_scenario(CFG)


def check_home(m):
    cfg = CFG
    d = mujoco.MjData(m)
    d.qpos[:] = scene.logical_to_internal(cfg.home, cfg)
    d.qvel[:] = 0.
    mujoco.mj_forward(m, d)
    singular = np.linalg.svd(scene.analytic_jacobian(np.asarray(cfg.home), cfg),
                             compute_uv=False)
    assert singular[-1] > .02, singular
    tool = d.site_xpos[m.site("tool").id]
    assert tool[2] > cfg.table_z + cfg.tool_radius + .01, tool
    assert d.ncon == 0, d.ncon
    return {"home_logical_rad": list(cfg.home), "home_tool_position_m": tool.tolist(),
            "home_jacobian_singular_values": singular.tolist(),
            "home_contacts": int(d.ncon), "non_singular": True}


def check_collision_audit(m):
    cfg = CFG
    root = ET.fromstring(scene.generated_xml(cfg))
    exclusions = set()
    for element in root.findall("./contact/exclude"):
        exclusions.add(frozenset((element.get("body1"), element.get("body2"))))
    expected = {
        frozenset(("base_link", "shoulder_link")),
        frozenset(("shoulder_link", "elbow_link")),
        frozenset(("yaw_hub", "base_link")),
        frozenset(("elbow_link", "tool_payload")),
    }
    assert exclusions == expected, exclusions
    table = m.geom("tabletop")
    marker = m.geom("task_marker")
    hub = m.geom("yaw_hub_geom")
    assert m.geom_contype[table.id] and m.geom_conaffinity[table.id]
    assert m.geom_contype[hub.id] and m.geom_conaffinity[hub.id]
    assert not m.geom_contype[marker.id] and not m.geom_conaffinity[marker.id]
    assert frozenset(("base_link", "elbow_link")) not in exclusions
    assert frozenset(("yaw_hub", "shoulder_link")) not in exclusions
    assert m.site("tool").bodyid == m.body("tool_payload").id

    d = mujoco.MjData(m)
    clearances = []
    contacts = 0
    for logical in (cfg.home, (0.0417, -0.025, -1.13)):
        d.qpos[:] = scene.logical_to_internal(logical, cfg)
        d.qvel[:] = 0.
        mujoco.mj_forward(m, d)
        contacts = max(contacts, int(d.ncon))
        for g1 in range(m.ngeom):
            name1 = m.geom(g1).name
            body1 = m.body(m.geom_bodyid[g1]).name
            if name1 in ("tabletop", "task_marker"):
                continue
            if not m.geom_contype[g1] or not m.geom_conaffinity[g1]:
                continue
            for g2 in range(g1 + 1, m.ngeom):
                name2 = m.geom(g2).name
                body2 = m.body(m.geom_bodyid[g2]).name
                if body1 == body2 or name2 in ("tabletop", "task_marker"):
                    continue
                if not m.geom_contype[g2] or not m.geom_conaffinity[g2]:
                    continue
                if frozenset((body1, body2)) in exclusions:
                    continue
                distance = float(mujoco.mj_geomDistance(m, d, g1, g2, 2.0, None))
                clearances.append({"pair": [name1, name2], "distance_m": distance})
    minimum = min(item["distance_m"] for item in clearances)
    assert minimum > .002, (minimum, min(clearances, key=lambda x: x["distance_m"]))
    return {"exclusions": sorted([sorted(pair) for pair in exclusions]),
            "table_and_yaw_hub_collision_active": True,
            "marker_noncolliding": True,
            "tool_site_body": "tool_payload",
            "minimum_nonadjacent_clearance_m": minimum,
            "checked_poses": [list(cfg.home), [0.0417, -0.025, -1.13]],
            "contacts_at_checked_poses": contacts,
            "broad_exclusion_note": "adjacent rolling-wheel pair exclusions are body-wide; "
                                    "nonadjacent clearance is tested separately, not every "
                                    "possible self-contact"}


def check_counts(m):
    expected_joints = ["yaw", "shoulder_a", "shoulder_b", "elbow_a", "elbow_b"]
    expected_actuators = ["yaw_target", "shoulder_target", "elbow_target"]
    assert m.nq == 5 and m.neq == 2 and m.nu == 3, (m.nq, m.neq, m.nu)
    assert [m.joint(i).name for i in range(m.njnt)] == expected_joints
    assert [m.actuator(i).name for i in range(m.nu)] == expected_actuators
    assert m.opt.gravity.tolist() == [0., 0., -9.81]
    assert abs(m.opt.timestep - CFG.timestep) < 1e-15
    assert m.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    camera = m.camera("overhead")
    close(m.cam_pos[camera.id], CFG.camera_pos, 1e-12, "camera position")
    assert abs(float(m.cam_fovy[camera.id]) - CFG.camera_fovy) < 1e-12
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    identity = np.eye(3)
    close(d.cam_xmat[camera.id].reshape(3, 3), identity, 1e-12, "camera axes")
    base = m.body("base_link")
    base_rotation = d.xmat[base.id].reshape(3, 3)
    close(base_rotation @ np.array([0., 0., 1.]), [0., -1., 0.], 1e-12,
          "local +Z maps to world -Y")
    close(base_rotation @ np.array([0., 1., 0.]), [0., 0., 1.], 1e-12,
          "local +Y maps to world +Z")
    close(d.xpos[base.id], [0., 0., CFG.shoulder_height], 1e-12, "base height")
    return {"internal_qpos": m.nq, "actuators": m.nu, "equalities": m.neq,
            "joint_names": expected_joints, "actuator_order": expected_actuators,
            "gravity": m.opt.gravity.tolist(), "timestep": float(m.opt.timestep),
            "camera_pos": m.cam_pos[camera.id].tolist(),
            "camera_fovy_deg": float(m.cam_fovy[camera.id]),
            "camera_axes": d.cam_xmat[camera.id].reshape(3, 3).tolist(),
            "local_z_maps_to_world": (base_rotation @ np.array([0., 0., 1.])).tolist(),
            "base_position_m": d.xpos[base.id].tolist()}


def contact_metrics(m, d, cfg=CFG):
    gaps, slips = [], []
    for i, joint in enumerate(cfg.joints):
        parent = "base_link" if i == 0 else f"{cfg.joints[i - 1].name}_link"
        parent_id = m.body(parent).id
        child_id = m.body(f"{joint.name}_link").id
        if i == 0:
            proximal = d.xpos[parent_id].copy()
        else:
            proximal = d.xpos[parent_id] + d.xmat[parent_id].reshape(3, 3) @ np.array(
                [cfg.joints[i - 1].link_length, 0., 0.])
        distal = d.xpos[child_id].copy()
        delta = distal - proximal
        gaps.append(abs(float(np.linalg.norm(delta)) - joint.spacing))
        point = proximal + joint.proximal_radius * delta / np.linalg.norm(delta)
        velocities = []
        for body_id in (parent_id, child_id):
            jp = np.zeros((3, m.nv))
            jr = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jp, jr, point, body_id)
            velocities.append(jp @ d.qvel)
        slips.append(float(np.linalg.norm(velocities[1] - velocities[0])))
    return max(gaps), max(slips)


def check_kinematics(m):
    d = mujoco.MjData(m)
    cfg = CFG
    g = scene.mapping(cfg)
    max_position = max_analytic_jac = max_mujoco_jac = max_gap = max_slip = max_roundtrip = 0.
    poses = 0
    yaw_values = np.linspace(-cfg.yaw_limit + .01, cfg.yaw_limit - .01, 7)
    shoulder_values = np.linspace(-cfg.joints[0].bend_limit + .01,
                                  cfg.joints[0].bend_limit - .01, 9)
    elbow_values = np.linspace(-cfg.joints[1].bend_limit + .01,
                               cfg.joints[1].bend_limit - .01, 9)
    for yaw in yaw_values:
        for shoulder in shoulder_values:
            for elbow in elbow_values:
                logical = np.array([yaw, shoulder, elbow])
                d.qpos[:] = scene.logical_to_internal(logical, cfg)
                d.qvel[:] = g @ np.array([.7, -.4, .3])
                mujoco.mj_forward(m, d)
                tool = d.site_xpos[m.site("tool").id].copy()
                analytic = scene.analytic_fk(logical, cfg)
                max_position = max(max_position, float(np.linalg.norm(tool - analytic)))
                max_roundtrip = max(max_roundtrip,
                                    float(np.max(np.abs(scene.internal_to_logical(d.qpos, cfg) - logical))))
                jp = np.zeros((3, m.nv))
                jr = np.zeros((3, m.nv))
                mujoco.mj_jacSite(m, d, jp, jr, m.site("tool").id)
                mapped = jp @ g
                analytic_jac = scene.analytic_jacobian(logical, cfg)
                finite = np.zeros((3, 3))
                step = 1e-6
                for k in range(3):
                    delta = np.zeros(3)
                    delta[k] = step
                    plus = np.zeros(3)
                    minus = np.zeros(3)
                    d.qpos[:] = scene.logical_to_internal(logical + delta, cfg)
                    mujoco.mj_forward(m, d)
                    plus[:] = d.site_xpos[m.site("tool").id]
                    d.qpos[:] = scene.logical_to_internal(logical - delta, cfg)
                    mujoco.mj_forward(m, d)
                    minus[:] = d.site_xpos[m.site("tool").id]
                    finite[:, k] = (plus - minus) / (2. * step)
                max_analytic_jac = max(max_analytic_jac,
                                       float(np.max(np.abs(analytic_jac - finite))))
                max_mujoco_jac = max(max_mujoco_jac,
                                     float(np.max(np.abs(mapped - finite))))
                d.qpos[:] = scene.logical_to_internal(logical, cfg)
                d.qvel[:] = g @ np.array([.7, -.4, .3])
                mujoco.mj_forward(m, d)
                gap, slip = contact_metrics(m, d, cfg)
                max_gap = max(max_gap, gap)
                max_slip = max(max_slip, slip)
                poses += 1
    assert max_position < 1e-11, max_position
    assert max_analytic_jac < 2e-8, max_analytic_jac
    assert max_mujoco_jac < 2e-8, max_mujoco_jac
    assert max_gap < 1e-12, max_gap
    assert max_slip < 1e-12, max_slip
    assert max_roundtrip < 1e-15, max_roundtrip
    return {"poses": poses, "max_tool_position_error_m": max_position,
            "max_analytic_jacobian_vs_finite_difference": max_analytic_jac,
            "max_mujoco_site_jacobian_vs_finite_difference": max_mujoco_jac,
            "max_rolling_center_gap_m": max_gap,
            "max_contact_point_velocity_m_s": max_slip,
            "max_mapping_roundtrip_rad": max_roundtrip}


def check_actuation(m):
    cfg = CFG
    d = mujoco.MjData(m)
    logical = np.array([.3, -.4, .2])
    d.qpos[:] = scene.logical_to_internal(logical, cfg)
    d.qvel[:] = scene.mapping(cfg) @ np.array([.2, -.3, .5])
    d.ctrl[:] = [.1, -.2, -.25]
    mujoco.mj_forward(m, d)
    close(d.actuator_length, logical, 1e-14, "actuator lengths")
    close(d.actuator_velocity, [.2, -.3, .5], 1e-14, "actuator velocities")
    s = scene.observer(cfg)
    g = scene.mapping(cfg)
    moment = scene.actuator_moment(cfg)
    close(s @ g, np.eye(3), 1e-14, "S G = I")
    close(g.T @ moment.T, np.eye(3), 1e-14, "G.T actuator_moment.T = I")
    close(d.qfrc_actuator, moment.T @ d.actuator_force, 1e-12, "actuator moment")
    work = float(d.qfrc_actuator @ d.qvel - d.actuator_force @ d.actuator_velocity)
    assert abs(work) < 1e-12, work
    for i, joint in enumerate(cfg.joints):
        actuator = m.actuator(f"{joint.name}_target")
        close(m.actuator_ctrlrange[actuator.id], (-joint.bend_limit, joint.bend_limit),
              1e-12, "bend control range")
        close(m.actuator_forcerange[actuator.id], (-cfg.bend_force, cfg.bend_force),
              1e-12, "bend force range")
    yaw = m.actuator("yaw_target")
    close(m.actuator_ctrlrange[yaw.id], (-cfg.yaw_limit, cfg.yaw_limit),
          1e-12, "yaw control range")
    close(m.actuator_forcerange[yaw.id], (-cfg.yaw_force, cfg.yaw_force),
          1e-12, "yaw force range")
    return {"virtual_work_error_w": work, "actuator_moment_rows": moment.tolist(),
            "tendon_lengths_are_logical_bends": True,
            "force_and_control_ranges_pinned": True}


def check_masses_and_inertia(m):
    cfg = CFG
    expected = {"yaw_hub": cfg.yaw_hub_mass,
                "base_link": 0.,
                "shoulder_carrier": cfg.carrier_mass,
                "shoulder_link": cfg.joints[0].link_mass,
                "elbow_carrier": cfg.carrier_mass,
                "elbow_link": cfg.joints[1].link_mass,
                "tool_payload": cfg.tool_mass}
    actual = {}
    for name, mass in expected.items():
        body = m.body(name)
        actual[name] = float(m.body_mass[body.id])
        if abs(actual[name] - mass) > 1e-12:
            raise AssertionError(f"mass ownership {name}: {actual[name]} != {mass}")
    total = float(m.body_mass.sum())
    expected_total = cfg.yaw_hub_mass + 2. * cfg.carrier_mass + sum(
        j.link_mass for j in cfg.joints) + cfg.tool_mass
    close(total, expected_total, 1e-12, "total mass")
    inertia = {}
    for name in expected:
        body = m.body(name)
        diagonal = np.array(m.body_inertia[body.id], dtype=float)
        inertia[name] = diagonal.tolist()
        if expected[name] == 0.:
            continue
        eigenvalues = np.linalg.eigvalsh(np.diag(diagonal))
        assert np.all(eigenvalues > 0.), (name, eigenvalues)
        i1, i2, i3 = eigenvalues
        assert i1 + i2 >= i3 - 1e-15
        assert i2 + i3 >= i1 - 1e-15
        assert i3 + i1 >= i2 - 1e-15
    close(m.body_ipos[m.body("tool_payload").id], (0., 0., 0.), 1e-15, "payload COM")
    return {"body_masses_kg": actual, "total_mass_kg": total,
            "principal_diagonal_inertia_kg_m2": inertia,
            "positive_definite_and_triangle_inequality": True}


def check_projection(m):
    cam = scene.camera_calibration(CFG)
    points = np.array([[CFG.marker_xy[0], CFG.marker_xy[1], 0.],
                       [.45, .10, 0.], [.70, -.20, 0.]])
    u, v, valid = scene.project_points(points, cam)
    assert np.all(valid)
    point, ok = scene.unproject_to_plane(u, v, 0., cam)
    assert np.all(ok)
    close(point[:, :2], points[:, :2], 1e-10, "project/backproject")
    high = np.array([[cam.pos[0], cam.pos[1], cam.pos[2] + .1]])
    _, _, behind_valid = scene.project_points(high, cam)
    assert not bool(behind_valid[0])
    row_high = scene.project_points([[.45, .10, 0.]], cam)[1][0]
    row_low = scene.project_points([[.45, -.10, 0.]], cam)[1][0]
    assert row_high < cam.height / 2. < row_low
    parallel_rotation = ((1., 0., 0.), (0., 0., -1.), (0., 1., 0.))
    parallel = scene.CameraCalibration(cam.pos, parallel_rotation, cam.fovy,
                                       cam.width, cam.height)
    _, parallel_valid = scene.unproject_to_plane(cam.width / 2. - .5,
                                                 cam.height / 2. - .5, 0., parallel)
    assert not bool(parallel_valid)
    return {"positive_rays_valid": True, "behind_camera_rejected": True,
            "parallel_ray_rejected": True,
            "image_row_direction": "increasing row means decreasing world Y",
            "roundtrip_error_m": float(np.max(np.abs(point[:, :2] - points[:, :2])))}


def check_red_detection(m, render=False):
    cfg = CFG
    cam = scene.camera_calibration(cfg)
    d = mujoco.MjData(m)
    detections = []
    max_pixel_error = 0.
    renderer = mujoco.Renderer(m, height=cfg.image_height, width=cfg.image_width)
    try:
        for case in cfg.probes:
            reach.set_marker(m, case.marker_xy, cfg)
            d.qpos[:] = scene.logical_to_internal(cfg.home, cfg)
            d.qvel[:] = 0.
            mujoco.mj_forward(m, d)
            renderer.update_scene(d, camera="overhead")
            rgb = renderer.render()
            valid_rgb = rgb.copy()
            center, count, fit = scene.red_center(rgb, cfg, cam)
            if center is None:
                raise AssertionError(f"marker not detected for {case.name}: {fit}")
            expected_u, expected_v, _ = scene.project_points(
                [[case.marker_xy[0], case.marker_xy[1], 0.]], cam)
            error = math.hypot(center[1] - expected_u[0], center[0] - expected_v[0])
            max_pixel_error = max(max_pixel_error, error)
            assert error < 1.5, (case.name, center, expected_u, expected_v, fit)
            detections.append({"case": case.name, "centroid_row_col": list(center),
                               "red_pixels": count, "fit": fit,
                               "pixel_error": error})
        reach.set_marker(m, (2.0, 2.0), cfg)
        mujoco.mj_forward(m, d)
        renderer.update_scene(d, camera="overhead")
        absent = renderer.render()
        absent_center, absent_count, _ = scene.red_center(absent, cfg, cam)
        assert absent_center is None, (absent_center, absent_count)
        policy = reach.ReferenceReacher(scene.policy_config(cfg))
        joints = np.asarray(cfg.home, dtype=float)
        first_target = policy.observe(valid_rgb, joints, 0.)
        held_target = policy.observe(np.zeros_like(valid_rgb), joints, .1)
        close(held_target, first_target, 1e-15, "missing-marker hold")
        assert np.linalg.norm(first_target - np.asarray(cfg.home)) > 1e-3
    finally:
        renderer.close()
    if render:
        reach.set_marker(m, cfg.marker_xy, cfg)
        d.qpos[:] = scene.logical_to_internal(cfg.home, cfg)
        mujoco.mj_forward(m, d)
        with mujoco.Renderer(m, height=cfg.image_height, width=cfg.image_width) as r:
            r.update_scene(d, camera="overhead")
            image = r.render()
        from PIL import Image
        Image.fromarray(image).save(RESULTS / "marker_extraction.png")
    return {"detections": detections, "max_pixel_error": max_pixel_error,
            "absent_marker_rejected": True,
            "missing_marker_holds_last_target": True,
            "held_target_rad": held_target.tolist(),
            "first_target_rad": first_target.tolist()}


def check_governor():
    cfg = CFG
    rng = np.random.default_rng(7)
    limits = np.array([cfg.yaw_limit, cfg.joints[0].bend_limit, cfg.joints[1].bend_limit])
    governor = reach.Governor(cfg.home, cfg)
    dt = cfg.timestep * cfg.control_decimation
    commands = []
    target = np.array(cfg.home)
    max_speed = 0.
    max_accel = 0.
    for step in range(4000):
        if step % 80 == 0:
            target = rng.uniform(-.9, .9, size=3) * limits
        command = governor.step(target, dt)
        commands.append(command.copy())
        if step:
            velocity = (commands[-1] - commands[-2]) / dt
            acceleration = (commands[-1] - 2. * commands[-2] + commands[-3]) / (dt * dt) if step > 1 else np.zeros(3)
            max_speed = max(max_speed, float(np.max(np.abs(velocity))))
            max_accel = max(max_accel, float(np.max(np.abs(acceleration))))
    assert max_speed <= cfg.speed_limit + 1e-9, max_speed
    assert max_accel <= cfg.accel_limit + 1e-6, max_accel
    return {"command_speed_limit_rad_s": cfg.speed_limit,
            "command_accel_limit_rad_s2": cfg.accel_limit,
            "observed_max_speed_rad_s": max_speed,
            "observed_max_accel_rad_s2": max_accel}


def check_loaded_dynamics():
    cfg = CFG
    m = reach.build_model(cfg)
    d = mujoco.MjData(m)
    d.qpos[:] = scene.logical_to_internal(cfg.home, cfg)
    d.qvel[:] = 0.
    d.ctrl[:] = cfg.home
    mujoco.mj_forward(m, d)
    governor = reach.Governor(cfg.home, cfg)
    maximum = {"force": 0., "speed": 0., "acceleration": 0., "tracking": 0.,
               "fk": 0., "coupling": 0., "contacts": 0}
    warnings = np.zeros_like(d.warning.number)
    steps = round(8. / cfg.timestep)
    for step in range(steps):
        t = d.time
        target = np.array([.30 * np.sin(2. * np.pi * .10 * t),
                           .06 * np.sin(2. * np.pi * .15 * t),
                           -.9 + .06 * np.sin(2. * np.pi * .13 * t)])
        if step % cfg.control_decimation == 0:
            d.ctrl[:] = governor.step(target, cfg.timestep * cfg.control_decimation)
        d.xfrc_applied[:] = 0.
        if 3. <= t < 4.:
            d.xfrc_applied[m.body("elbow_link").id, 2] = -2.
        d.qfrc_applied[:] = 0.
        if 5. <= t < 5.4:
            d.qfrc_applied[:] = [0., -.4, .4, -.2, .2]
        mujoco.mj_step(m, d)
        warnings += d.warning.number
        logical = scene.internal_to_logical(d.qpos, cfg)
        maximum["force"] = max(maximum["force"], float(np.max(np.abs(d.actuator_force))))
        maximum["speed"] = max(maximum["speed"], float(np.max(np.abs(d.qvel))))
        maximum["acceleration"] = max(maximum["acceleration"],
                                       float(np.max(np.abs(d.qacc))))
        maximum["tracking"] = max(maximum["tracking"],
                                  float(np.max(np.abs(logical - target))))
        maximum["fk"] = max(maximum["fk"], float(np.linalg.norm(
            d.site_xpos[m.site("tool").id] - scene.analytic_fk(logical, cfg))))
        maximum["coupling"] = max(maximum["coupling"], float(max(
            abs(d.qpos[2] - cfg.joints[0].ratio * d.qpos[1]),
            abs(d.qpos[4] - cfg.joints[1].ratio * d.qpos[3]))))
        maximum["contacts"] = max(maximum["contacts"], int(d.ncon))
    assert not np.any(warnings), warnings
    assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
    assert maximum["force"] <= max(cfg.bend_force, cfg.yaw_force) + 1e-9
    assert maximum["speed"] <= cfg.speed_limit + 1e-9
    assert maximum["acceleration"] < 150.
    assert maximum["tracking"] < .08
    assert maximum["fk"] < 1e-3
    assert maximum["coupling"] < 1e-3
    return {"simulated_seconds": float(d.time), "warning_counts": warnings.tolist(),
            "maxima": maximum,
            "load": "gravity, 2 N downward elbow load during 3-4 s, "
                    "antagonistic hinge torques during 5.0-5.4 s",
            "commands_from_acceleration_limited_governor": True}


def check_constraints():
    cfg = CFG
    source = scene.generated_xml(cfg)
    results = {}
    for mutation in ("intact", "remove_equalities", "reverse_equalities"):
        m = mujoco.MjModel.from_xml_string(source)
        if mutation == "reverse_equalities":
            m.eq_data[:, 1] *= -1.
        d = mujoco.MjData(m)
        if mutation == "remove_equalities":
            d.eq_active[:] = 0
        d.qpos[:] = scene.logical_to_internal(cfg.home, cfg)
        d.qvel[:] = 0.
        d.ctrl[:] = cfg.home
        mujoco.mj_forward(m, d)
        maximum = 0.
        for _ in range(3000):
            d.qfrc_applied[:] = [0., -.4, .4, -.2, .2]
            mujoco.mj_step(m, d)
            coupling = max(abs(d.qpos[2] - cfg.joints[0].ratio * d.qpos[1]),
                           abs(d.qpos[4] - cfg.joints[1].ratio * d.qpos[3]))
            maximum = max(maximum, float(coupling))
        results[mutation] = {"max_coupling_rad": maximum,
                             "warning_counts": d.warning.number.tolist()}
    assert results["intact"]["max_coupling_rad"] < 1e-3
    assert results["remove_equalities"]["max_coupling_rad"] > .02
    assert results["reverse_equalities"]["max_coupling_rad"] > .02
    return results


def _augment_urdf(imported_xml, directory):
    root = ET.parse(imported_xml).getroot()
    root.set("model", CFG.name)
    option = ET.Element("option", timestep=scene.numbers((CFG.timestep,)),
                        gravity="0 0 -9.81", integrator="implicitfast",
                        iterations="100", tolerance="1e-10")
    root.insert(0, option)
    default = ET.Element("default")
    ET.SubElement(default, "joint", damping=".03", armature=".0001")
    root.insert(list(root).index(root.find("worldbody")), default)
    world = root.find("worldbody")
    ET.SubElement(world, "geom", name="tabletop", type="plane", size="2 2 .01",
                  pos="0 0 0", rgba=".18 .20 .23 1", friction=".9 .01 .001")
    ET.SubElement(world, "light", name="key", pos="0 0 2.5", dir="0 0 -1",
                  directional="true", diffuse=".9 .9 .9")
    ET.SubElement(world, "camera", name="overhead", pos=scene.numbers(CFG.camera_pos),
                  xyaxes="1 0 0 0 1 0", fovy=str(CFG.camera_fovy))
    tool = root.find(".//body[@name='tool']")
    ET.SubElement(tool, "site", name="tool", size=".018")
    contact = ET.Element("contact")
    for body1, body2 in (("yaw_hub", "base_link"), ("base_link", "shoulder_link"),
                         ("shoulder_link", "elbow_link"), ("elbow_link", "tool")):
        ET.SubElement(contact, "exclude", body1=body1, body2=body2)
    root.insert(list(root).index(root.find("equality")), contact)
    for equality in root.findall("./equality/joint"):
        equality.set("solref", ".002 1")
        equality.set("solimp", ".9999 .9999 .001")
    tendons = ET.Element("tendon")
    for joint in CFG.joints:
        fixed = ET.SubElement(tendons, "fixed", name=f"{joint.name}_bend")
        for suffix in ("a", "b"):
            ET.SubElement(fixed, "joint", joint=f"{joint.name}_{suffix}", coef="1")
    root.append(tendons)
    actuators = ET.Element("actuator")
    ET.SubElement(actuators, "position", name="yaw_target", joint="yaw",
                  kp=scene.numbers((CFG.yaw_kp,)), kv=scene.numbers((CFG.yaw_kv,)),
                  ctrlrange=scene.numbers((-CFG.yaw_limit, CFG.yaw_limit)),
                  forcerange=scene.numbers((-CFG.yaw_force, CFG.yaw_force)))
    for joint in CFG.joints:
        ET.SubElement(actuators, "position", name=f"{joint.name}_target",
                      tendon=f"{joint.name}_bend",
                      kp=scene.numbers((CFG.bend_kp,)), kv=scene.numbers((CFG.bend_kv,)),
                      ctrlrange=scene.numbers((-joint.bend_limit, joint.bend_limit)),
                      forcerange=scene.numbers((-CFG.bend_force, CFG.bend_force)))
    root.append(actuators)
    path = directory / "urdf_control_augmented.xml"
    scene.write_xml(path, root)
    return path


def check_urdf(native_model, native_counts):
    cfg = CFG
    directory = RESULTS / "model"
    scene.write_generated(directory, cfg)
    urdf_path = directory / "arm.urdf"
    robot = yourdfpy.URDF.load(str(urdf_path))
    actuated = set(robot.actuated_joint_names)
    assert actuated == {"yaw", "shoulder_a", "elbow_a"}, actuated
    mimic = [name for name in robot.joint_names
             if getattr(robot.joint_map[name], "mimic", None) is not None]
    assert set(mimic) == {"shoulder_b", "elbow_b"}, mimic
    hinge_names = [name for name in robot.joint_names
                   if robot.joint_map[name].type in ("revolute", "continuous")]
    assert len(hinge_names) == 5, hinge_names

    imported = mujoco.MjModel.from_xml_path(str(urdf_path))
    assert imported.nq == 5 and imported.neq == 2 and imported.nu == 0
    assert [imported.joint(i).name for i in range(imported.njnt)] == [
        "yaw", "shoulder_a", "shoulder_b", "elbow_a", "elbow_b"]
    for i, joint in enumerate(cfg.joints):
        assert imported.joint(int(imported.eq_obj1id[i])).name == f"{joint.name}_b"
        assert imported.joint(int(imported.eq_obj2id[i])).name == f"{joint.name}_a"
        close(imported.eq_data[i, :5], [0., joint.ratio, 0., 0., 0.], 1e-12,
              "URDF mimic equality")

    max_fk = 0.
    for yaw, shoulder, elbow in ((0., 0., -.9), (.3, -.2, -.8),
                                 (-.5, .4, -1.2), (.7, -.7, .5)):
        internal = scene.logical_to_internal([yaw, shoulder, elbow], cfg)
        robot.update_cfg({"yaw": yaw, "shoulder_a": internal[1], "elbow_a": internal[3]})
        expected = scene.analytic_fk([yaw, shoulder, elbow], cfg)
        expected_shoulder, expected_elbow = scene.analytic_links(
            [yaw, shoulder, elbow], cfg)
        actual = np.array([
            robot.get_transform("shoulder_link")[:3, 3],
            robot.get_transform("elbow_link")[:3, 3],
            robot.get_transform("tool")[:3, 3],
        ])
        max_fk = max(max_fk, float(np.max(np.abs(actual - np.array(
            [expected_shoulder, expected_elbow, expected])))))
    assert max_fk < 1e-10, max_fk

    native_mass = float(native_model.body_mass.sum())
    imported_mass = float(imported.body_mass.sum())
    close(imported_mass, native_mass, 1e-12, "URDF total mass")
    imported_bodies = {imported.body(i).name: float(imported.body_mass[i])
                       for i in range(imported.nbody)}
    for name in ("yaw_hub", "base_link", "shoulder_carrier", "shoulder_link",
                 "elbow_carrier", "elbow_link"):
        assert abs(imported_bodies[name] - {
            "yaw_hub": cfg.yaw_hub_mass, "base_link": 0.,
            "shoulder_carrier": cfg.carrier_mass,
            "shoulder_link": cfg.joints[0].link_mass,
            "elbow_carrier": cfg.carrier_mass,
            "elbow_link": cfg.joints[1].link_mass}[name]) < 1e-12
    assert abs(imported_bodies["tool"] - cfg.tool_mass) < 1e-12

    mujoco.mj_saveLastXML(str(directory / "urdf_import.xml"), imported)
    augmented_path = _augment_urdf(directory / "urdf_import.xml", directory)
    augmented = mujoco.MjModel.from_xml_path(str(augmented_path))
    assert augmented.nq == native_counts["internal_qpos"]
    assert augmented.neq == native_counts["equalities"]
    assert augmented.nu == native_counts["actuators"]
    d = mujoco.MjData(augmented)
    d.qpos[:] = scene.logical_to_internal(cfg.home, cfg)
    d.qvel[:] = 0.
    d.ctrl[:] = cfg.home
    mujoco.mj_forward(augmented, d)
    governor = reach.Governor(cfg.home, cfg)
    max_tool = max_force = max_speed = max_coupling = 0.
    warnings = np.zeros_like(d.warning.number)
    for step in range(round(3. / cfg.timestep)):
        target = np.array([.20 * np.sin(2. * np.pi * .1 * d.time),
                           .05 * np.sin(2. * np.pi * .12 * d.time),
                           -.9 + .05 * np.sin(2. * np.pi * .11 * d.time)])
        if step % cfg.control_decimation == 0:
            d.ctrl[:] = governor.step(target, cfg.timestep * cfg.control_decimation)
        mujoco.mj_step(augmented, d)
        warnings += d.warning.number
        logical = scene.internal_to_logical(d.qpos, cfg)
        max_tool = max(max_tool, float(np.linalg.norm(
            d.site_xpos[augmented.site("tool").id] - scene.analytic_fk(logical, cfg))))
        max_force = max(max_force, float(np.max(np.abs(d.actuator_force))))
        max_speed = max(max_speed, float(np.max(np.abs(d.qvel))))
        max_coupling = max(max_coupling, float(max(
            abs(d.qpos[2] - cfg.joints[0].ratio * d.qpos[1]),
            abs(d.qpos[4] - cfg.joints[1].ratio * d.qpos[3]))))
    assert not np.any(warnings), warnings
    assert max_tool < 1e-3, max_tool
    assert max_coupling < 1e-3, max_coupling
    assert max_force <= max(cfg.bend_force, cfg.yaw_force) + 1e-9
    assert max_speed <= cfg.speed_limit + 1e-9
    return {"yourdfpy_actuated_joints": sorted(actuated), "mimic_joints": sorted(mimic),
            "hinge_joints": len(hinge_names), "independent_coordinates": 3,
            "direct_import": {"nq": imported.nq, "neq": imported.neq, "nu": imported.nu},
            "maximum_yourdfpy_fk_error_m": max_fk,
            "imported_total_mass_kg": imported_mass,
            "augmented": {"nq": augmented.nq, "neq": augmented.neq, "nu": augmented.nu,
                          "loaded_simulated_seconds": float(d.time),
                          "max_tool_fk_error_m": max_tool,
                          "max_equality_coupling_rad": max_coupling,
                          "max_actuator_force_nm": max_force,
                          "max_joint_speed_rad_s": max_speed,
                          "warning_counts": warnings.tolist()},
            "representation_differences": [
                "URDF uses box/cylinder collision geometry; native MJCF uses capsules",
                "URDF has a fixed yaw_mount body and a fixed tool body",
                "URDF-loaded dynamics started above the table; contact topology is not identical",
            ]}


def render_preview():
    m = reach.build_model(CFG)
    d = mujoco.MjData(m)
    reach.set_marker(m, CFG.marker_xy, CFG)
    d.qpos[:] = scene.logical_to_internal(CFG.home, CFG)
    mujoco.mj_forward(m, d)
    from PIL import Image, ImageDraw
    frames = []
    with mujoco.Renderer(m, height=CFG.image_height, width=CFG.image_width) as renderer:
        for label, logical in (("home", CFG.home), ("above marker", (0.0417, -0.025, -1.13))):
            d.qpos[:] = scene.logical_to_internal(logical, CFG)
            mujoco.mj_forward(m, d)
            renderer.update_scene(d, camera="overhead")
            image = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, CFG.image_width, 36), fill=(12, 18, 26))
            draw.text((8, 6), f"sim_rolling_reach_v1 | {label}", fill="white")
            frames.append(image)
    preview = Image.new("RGB", (CFG.image_width * 2, CFG.image_height))
    for i, frame in enumerate(frames):
        preview.paste(frame, (i * CFG.image_width, 0))
    preview.save(RESULTS / "render" / "verify_preview.png")
    frames[0].save(RESULTS / "render" / "verify_home.png")
    frames[1].save(RESULTS / "render" / "verify_goal.png")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args(argv)
    RESULTS.mkdir(parents=True, exist_ok=True)
    for subdir in ("model", "render"):
        (RESULTS / subdir).mkdir(exist_ok=True)

    frozen = freeze_cases()
    assert not any(case["excluded"] for case in frozen["cases"]), frozen["cases"]
    m = reach.build_model(CFG)
    counts = check_counts(m)
    report = {
        "scope": "new self-contained rolling-contact reaching simulator",
        "evidence_classes": {
            "model_geometry_and_dynamics": "actual MuJoCo simulation",
            "reference_image_reaching": "actual rendered RGB, hand-coded baseline",
            "urdf_import": "actual yourdfpy and MuJoCo import",
            "dataset_fixtures": "not included",
            "sdk_mocks": "not included",
            "learned_or_hosted_policy": "not included",
            "physical_hardware": "not included",
        },
        "versions": _versions(),
        "config_validation": check_config(),
        "frozen_cases": frozen,
        "model_counts": counts,
        "home_pose": check_home(m),
        "collision_audit": check_collision_audit(m),
        "kinematics": check_kinematics(m),
        "actuation": check_actuation(m),
        "masses_and_inertia": check_masses_and_inertia(m),
        "projection": check_projection(m),
        "red_detection": check_red_detection(m, render=args.render),
        "governor": check_governor(),
        "loaded_dynamics": check_loaded_dynamics(),
        "constraint_controls": check_constraints(),
        "urdf": check_urdf(m, counts),
    }

    reach_report = reach.run_suite(episodes=len(CFG.probes), seed=0, render=args.render)
    for attempt in reach_report["attempts"] + reach_report["shift_attempts"]:
        assert attempt["success"], attempt["case"]
        assert attempt["final_error_m"] < CFG.success_tolerance, attempt
        assert attempt["success_time_s"] < CFG.timeout_seconds, attempt
        assert not any(attempt["warning_counts"]), attempt
        assert attempt["max_control_speed_rad_s"] <= CFG.speed_limit + 1e-9, attempt
        assert attempt["max_control_accel_rad_s2"] <= CFG.accel_limit + 1e-6, attempt
        assert attempt["max_actuator_force_nm"] <= max(CFG.bend_force, CFG.yaw_force) + 1e-9
    for attempt in reach_report["control_attempts"]:
        assert not attempt["success"], attempt
        assert not any(attempt["warning_counts"]), attempt
    for live, frozen_attempt in zip(reach_report["shift_attempts"],
                                    reach_report["control_attempts"][::2]):
        change = np.linalg.norm(np.array(live["command_after_shift"]) -
                                np.array(live["command_before_shift"]))
        stale = np.linalg.norm(np.array(frozen_attempt["command_after_shift"]) -
                               np.array(frozen_attempt["command_before_shift"]))
        assert change > .05, (live["case"], change)
        assert stale < 1e-3, (frozen_attempt["case"], stale)
    reach_report["adaptation_checks"] = {
        "live_shift_min_command_change_rad": min(
            float(np.linalg.norm(np.array(a["command_after_shift"]) -
                                 np.array(a["command_before_shift"])))
            for a in reach_report["shift_attempts"]),
        "frozen_image_max_command_change_rad": max(
            float(np.linalg.norm(np.array(a["command_after_shift"]) -
                                 np.array(a["command_before_shift"])))
            for a in reach_report["control_attempts"][::2]),
        "frozen_image_and_hold_fail_as_expected": True,
    }
    for held in reach_report["control_attempts"][1::2]:
        close(held["command_after_shift"], held["command_before_shift"], 1e-12, "hold command remains unchanged")
    report["reference_reaching"] = reach_report
    if args.render:
        render_preview()
        report["render"] = {"preview": "results/render/verify_preview.png",
                            "home": "results/render/verify_home.png",
                            "goal": "results/render/verify_goal.png",
                            "motion_gif": "results/render/*_motion.gif"}
    report["sha256"] = {
        name: hashlib.sha256((HERE / name).read_bytes()).hexdigest()
        for name in ("scene.py", "reach.py", "verify.py", "README.md", "../rolling_contact_arm/model.py")
        if (HERE / name).exists()
    }
    report["verdict"] = "PASS"
    (RESULTS / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"verdict": report["verdict"],
                      "poses": report["kinematics"]["poses"],
                      "probes": len(report["reference_reaching"]["attempts"]),
                      "shift_cases": len(report["reference_reaching"]["shift_attempts"]),
                      "controls": len(report["reference_reaching"]["control_attempts"]),
                      "output": "results/verification.json"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
