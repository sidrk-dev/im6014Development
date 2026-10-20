"""Run geometry, importer, dynamics, and constraint-loss experiments."""

import argparse
from dataclasses import replace
import hashlib
import importlib.metadata
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yourdfpy

from model import (BASE, HERE, JOINTS, TIMESTEP, generate, internal_to_logical,
                   logical_to_internal, mjcf, urdf, write_xml)

OUTPUT = HERE / "results"


def direction(angle):
    return np.array([np.cos(angle), np.sin(angle), 0.])


def reference(bends, joints=JOINTS):
    shoulder, elbow = bends
    s, e = joints
    carrier_s = shoulder * s.distal_radius / s.spacing
    carrier_e = elbow * e.distal_radius / e.spacing
    upper = BASE + s.spacing * direction(carrier_s)
    pivot_e = upper + s.link_length * direction(shoulder)
    forearm = pivot_e + e.spacing * direction(shoulder + carrier_e)
    tool = forearm + e.link_length * direction(shoulder + elbow) + [0, 0, .025]
    return upper, forearm, tool


def mapping(joints=JOINTS):
    matrix = np.zeros((4, 2))
    for i, j in enumerate(joints):
        matrix[2*i:2*i+2, i] = [1/(1+j.ratio), j.ratio/(1+j.ratio)]
    return matrix


def contact_metrics(m, d, joints=JOINTS):
    gaps, slips = [], []
    for i, j in enumerate(joints):
        proximal_id = m.body("base_link" if i == 0 else f"{joints[i-1].name}_link").id
        distal_id = m.body(f"{j.name}_link").id
        proximal = (BASE if i == 0 else d.xpos[proximal_id] +
                    d.xmat[proximal_id].reshape(3, 3) @ np.array([joints[i-1].link_length, 0, 0]))
        delta = d.xpos[distal_id] - proximal
        gaps.append(abs(np.linalg.norm(delta) - j.spacing))
        point = proximal + j.proximal_radius * delta / np.linalg.norm(delta)
        velocities = []
        for body_id in (proximal_id, distal_id):
            jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jp, jr, point, body_id)
            velocities.append(jp @ d.qvel)
        slips.append(np.linalg.norm(velocities[1] - velocities[0]))
    return max(gaps), max(slips)


def check_geometry(m, u, joints=JOINTS):
    d = mujoco.MjData(m)
    matrix = mapping(joints)
    maxima = dict(mujoco_position_m=0., urdf_position_m=0., orientation_rad=0.,
                  jacobian_error=0., contact_gap_m=0., contact_slip_m_s=0., roundtrip_rad=0.)
    count = 0
    for qs in np.linspace(-joints[0].bend_limit, joints[0].bend_limit, 25):
        for qe in np.linspace(-joints[1].bend_limit, joints[1].bend_limit, 25):
            bends = np.array([qs, qe])
            internal = logical_to_internal(bends, joints)
            d.qpos[:] = internal
            d.qvel[:] = matrix @ np.array([.7, -.4])
            mujoco.mj_forward(m, d)
            u.update_cfg({f"{j.name}_a": internal[2*i] for i, j in enumerate(joints)})
            expected = reference(bends, joints)
            actual = (d.xpos[m.body("shoulder_link").id], d.xpos[m.body("elbow_link").id],
                      d.site_xpos[m.site("tool").id])
            urdf_positions = tuple(u.get_transform(name)[:3, 3] for name in ("shoulder_link", "elbow_link", "tool"))
            maxima["mujoco_position_m"] = max(maxima["mujoco_position_m"], max(np.linalg.norm(a-b) for a,b in zip(actual, expected)))
            maxima["urdf_position_m"] = max(maxima["urdf_position_m"], max(np.linalg.norm(a-b) for a,b in zip(urdf_positions, expected)))
            for name, angle in (("shoulder_link", qs), ("elbow_link", qs+qe)):
                actual_angle = np.arctan2(d.xmat[m.body(name).id, 3], d.xmat[m.body(name).id, 0])
                error = abs(np.arctan2(np.sin(actual_angle-angle), np.cos(actual_angle-angle)))
                maxima["orientation_rad"] = max(maxima["orientation_rad"], error)
            jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
            mujoco.mj_jacSite(m, d, jp, jr, m.site("tool").id)
            finite = np.zeros((3, 2))
            for k in range(2):
                step = np.eye(2)[k] * 1e-6
                finite[:, k] = (reference(bends+step, joints)[2]-reference(bends-step, joints)[2]) / 2e-6
            maxima["jacobian_error"] = max(maxima["jacobian_error"], float(np.max(abs(jp@matrix-finite))),
                                            float(np.max(abs(jr@matrix-np.array([[0,0],[0,0],[1,1]])))))
            gap, slip = contact_metrics(m, d, joints)
            maxima["contact_gap_m"] = max(maxima["contact_gap_m"], gap)
            maxima["contact_slip_m_s"] = max(maxima["contact_slip_m_s"], slip)
            maxima["roundtrip_rad"] = max(maxima["roundtrip_rad"], float(max(abs(internal_to_logical(internal)-bends))))
            count += 1
    assert max(maxima[k] for k in ("mujoco_position_m", "urdf_position_m", "orientation_rad")) < 1e-11, maxima
    assert maxima["jacobian_error"] < 2e-9, maxima
    assert maxima["contact_slip_m_s"] < 1e-12, maxima
    return {"poses": count, "max_errors": maxima}


def add_imported_control():
    imported = mujoco.MjModel.from_xml_path(str(HERE / "arm.urdf"))
    assert imported.nq == 4 and imported.neq == 2 and imported.nu == 0
    for i, j in enumerate(JOINTS):
        assert imported.joint(int(imported.eq_obj1id[i])).name == f"{j.name}_b"
        assert imported.joint(int(imported.eq_obj2id[i])).name == f"{j.name}_a"
        np.testing.assert_allclose(imported.eq_data[i, :5], [0, j.ratio, 0, 0, 0])
    mujoco.mj_saveLastXML(str(OUTPUT / "urdf_import.xml"), imported)
    root = ET.parse(OUTPUT / "urdf_import.xml").getroot()
    ET.SubElement(root, "option", timestep=str(TIMESTEP), gravity="0 -9.81 0", integrator="implicitfast",
                  iterations="100", tolerance="1e-10")
    source = mjcf()
    for tag in ("tendon", "actuator", "contact"):
        root.append(source.find(tag))
    for eq in root.findall("./equality/joint"):
        eq.set("solref", ".002 1")
        eq.set("solimp", ".9999 .9999 .001")
    for joint in root.findall(".//body/joint"):
        joint.set("damping", ".03")
        joint.set("armature", ".0001")
    last = root.find(".//body[@name='tool']")
    ET.SubElement(last, "site", name="tool", size=".01")
    path = OUTPUT / "urdf_with_control.xml"
    write_xml(path, root)
    return mujoco.MjModel.from_xml_path(str(path)), {
        "direct_import_nq": imported.nq, "direct_import_neq": imported.neq,
        "direct_import_nu": imported.nu, "mimic_preserved": True,
        "added": "logical-bend actuators, contact exclusions, solver settings, tool site; no added equality constraints"}


def check_actuation(m):
    d = mujoco.MjData(m)
    d.qpos[:] = logical_to_internal([.2, -.3])
    d.qvel[:] = mapping() @ np.array([.4, -.6])
    d.ctrl[:] = [.25, -.2]
    mujoco.mj_forward(m, d)
    np.testing.assert_allclose(d.actuator_length, internal_to_logical(d.qpos), atol=1e-14)
    np.testing.assert_allclose(d.actuator_velocity, internal_to_logical(d.qvel), atol=1e-14)
    force = d.actuator_force
    np.testing.assert_allclose(d.qfrc_actuator, [force[0], force[0], force[1], force[1]], atol=1e-14)
    error = abs(d.qfrc_actuator @ d.qvel - force @ internal_to_logical(d.qvel))
    assert error < 1e-13
    return {"position_and_velocity_transmission": "q=a+b", "virtual_work_error_w":float(error),
            "logical_torque_units":"Nm; one actuator force applied to both internal hinges"}


def dynamics(m, seconds=8., mutation=None):
    d = mujoco.MjData(m)
    if mutation == "remove_constraints":
        d.eq_active[:] = 0
    if mutation == "reverse_constraint":
        m.eq_data[:, 1] *= -1
    history = []
    maxima = dict(coupling_rad=0., contact_slip_m_s=0., contact_gap_m=0.,
                  tool_reference_error_m=0., tracking_rad=0., actuator_force_nm=0., actual_contacts=0)
    nsteps = round(seconds/m.opt.timestep)
    for step in range(nsteps):
        t = d.time
        target = np.array([.55*np.sin(2*np.pi*.13*t), .65*np.sin(2*np.pi*.17*t)])
        d.ctrl[:] = target
        d.xfrc_applied[:] = 0
        if 3 <= t < 4:
            d.xfrc_applied[m.body("elbow_link").id, 1] = -2.
        d.qfrc_applied[:] = 0
        if mutation or 5 <= t < 5.4:
            d.qfrc_applied[:] = [-.4, .4, -.2, .2]
        mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all(), "nonfinite dynamics"
        logical = internal_to_logical(d.qpos)
        coupling = [abs(d.qpos[2*i+1]-j.ratio*d.qpos[2*i]) for i,j in enumerate(JOINTS)]
        gap, slip = contact_metrics(m, d)
        error = np.linalg.norm(d.site_xpos[m.site("tool").id]-reference(logical)[2])
        maxima["coupling_rad"] = max(maxima["coupling_rad"], *coupling)
        maxima["contact_slip_m_s"] = max(maxima["contact_slip_m_s"], slip)
        maxima["contact_gap_m"] = max(maxima["contact_gap_m"], gap)
        maxima["tool_reference_error_m"] = max(maxima["tool_reference_error_m"], error)
        maxima["tracking_rad"] = max(maxima["tracking_rad"], float(max(abs(logical-target))))
        maxima["actuator_force_nm"] = max(maxima["actuator_force_nm"], float(max(abs(d.actuator_force))))
        maxima["actual_contacts"] = max(maxima["actual_contacts"], d.ncon)
        if step % 10 == 0:
            history.append({"t":d.time, "internal":d.qpos.tolist(), "target":target.tolist()})
    warning_counts = [int(v) for v in d.warning.number]
    result = {"simulated_seconds": float(d.time), "steps":nsteps, "metrics_checked_every_step":True, "maxima":maxima,
              "warning_counts":warning_counts, "load": "gravity -Y and constant antagonistic hinge torque" if mutation else
              "gravity -Y, 2N downward load during seconds 3-4, antagonistic hinge torque during seconds 5-5.4"}
    if mutation is None:
        assert not any(warning_counts), result
        assert maxima["coupling_rad"] < 1e-3, result
        assert maxima["tool_reference_error_m"] < 1e-4, result
        assert maxima["tracking_rad"] < .15, result
        assert maxima["contact_slip_m_s"] < .005, result
        assert maxima["actuator_force_nm"] <= 25.00001, result
    else:
        assert maxima["coupling_rad"] > .02, result
    return result, history


def render(m, history):
    from PIL import Image, ImageDraw
    d = mujoco.MjData(m)
    frames = []
    stride = max(1, round(.08 / (history[1]["t"]-history[0]["t"])))
    with mujoco.Renderer(m, height=480, width=720) as renderer:
        for entry in history[::stride]:
            d.qpos[:] = entry["internal"]
            mujoco.mj_forward(m, d)
            renderer.update_scene(d, camera="inspection")
            frame = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 0, 720, 54), fill=(13,18,26))
            draw.text((16, 10), "Illustrative rolling shoulder + elbow | MuJoCo dynamics", fill="white")
            bends = internal_to_logical(d.qpos)
            draw.text((16, 30), f"t={entry['t']:.2f}s   shoulder={bends[0]:+.3f} rad   elbow={bends[1]:+.3f} rad", fill=(160,200,240))
            frames.append(frame)
    frames[0].save(OUTPUT / "motion.gif", save_all=True, append_images=frames[1:], duration=80, loop=0)
    choices = [frames[i] for i in (0, len(frames)//3, 2*len(frames)//3)]
    preview = Image.new("RGB", (720*3, 480))
    for i, frame in enumerate(choices):
        preview.paste(frame, (720*i, 0))
    preview.save(OUTPUT / "preview.png")
    return {"gif_frames":len(frames), "backend":"MuJoCo offscreen renderer", "source":"recorded dynamic simulation qpos"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    OUTPUT.mkdir(exist_ok=True)
    generate()
    m = mujoco.MjModel.from_xml_path(str(HERE / "arm.xml"))
    u = yourdfpy.URDF.load(str(HERE / "arm.urdf"))
    assert u.actuated_joint_names == ["shoulder_a", "elbow_a"]
    results = {"scope":"illustrative ideal circular rolling; no hardware or learned policy validation",
               "versions":{name:importlib.metadata.version(name) for name in ("mujoco", "yourdfpy", "numpy", "Pillow")},
               "model_counts":{"internal_coordinates":m.nq,"rolling_equalities":m.neq,"logical_actuators":m.nu},
               "geometry":check_geometry(m, u), "actuation":check_actuation(m)}
    imported, results["urdf_import"] = add_imported_control()
    results["urdf_imported_geometry"] = check_geometry(imported, u)
    results["native_dynamics"], history = dynamics(m)
    results["urdf_imported_dynamics"], _ = dynamics(imported)
    unequal = (replace(JOINTS[0], proximal_radius=.06, distal_radius=.04),
               replace(JOINTS[1], proximal_radius=.05, distal_radius=.03))
    write_xml(OUTPUT / "unequal_radii.urdf", urdf(unequal))
    um = mujoco.MjModel.from_xml_string(ET.tostring(mjcf(unequal), encoding="unicode"))
    uu = yourdfpy.URDF.load(str(OUTPUT / "unequal_radii.urdf"))
    results["unequal_radius_geometry"] = check_geometry(um, uu, unequal)
    for mutation in ("remove_constraints", "reverse_constraint"):
        mm = mujoco.MjModel.from_xml_path(str(HERE / "arm.xml"))
        results[mutation], _ = dynamics(mm, seconds=1., mutation=mutation)
    for bad in ([np.nan, 0], [0, np.inf], [1.21, 0], [0], [0,0,0]):
        try:
            logical_to_internal(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"adapter accepted invalid input {bad}")
    results["adapter_invalid_targets_rejected"] = 5
    if args.render:
        results["render"] = render(m, history)
    results["sha256"] = {name:hashlib.sha256((HERE/name).read_bytes()).hexdigest()
                         for name in ("arm.xml", "arm.urdf", "manifest.json", "model.py", "verify.py")}
    results["verdict"] = "PASS"
    (OUTPUT / "verification.json").write_text(json.dumps(results, indent=2)+"\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
