"""Scene, configuration, composition, and analytic kinematics for sim_rolling_reach_v1."""

from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from examples.rolling_contact_arm import model as base_model  # noqa: E402


@dataclass(frozen=True)
class RollingPair:
    name: str
    proximal_radius: float
    distal_radius: float
    bend_limit: float
    link_length: float
    link_mass: float

    @property
    def ratio(self):
        return self.proximal_radius / self.distal_radius

    @property
    def spacing(self):
        return self.proximal_radius + self.distal_radius


@dataclass(frozen=True)
class Case:
    name: str
    marker_xy: tuple


@dataclass(frozen=True)
class ShiftCase:
    name: str
    start_xy: tuple
    shift_xy: tuple
    shift_time: float = 4.0


@dataclass(frozen=True)
class Scenario:
    name: str = "sim_rolling_reach_v1"
    revision: str = "rolling-reach-reference-v1"
    mechanical_name: str = "rolling_reach_arm"
    joints: tuple = (
        RollingPair("shoulder", .05, .05, 1.2, .32, .7),
        RollingPair("elbow", .04, .04, 1.4, .28, .4),
    )
    carrier_mass: float = .025
    yaw_hub_mass: float = .25
    yaw_hub_radius: float = .045
    yaw_hub_half_height: float = .04
    tool_mass: float = .10
    tool_radius: float = .016
    tool_lateral_offset: float = .025
    shoulder_height: float = .35
    yaw_limit: float = 1.2
    table_z: float = 0.
    marker_radius: float = .060
    marker_half_height: float = .0015
    goal_height: float = .04
    success_tolerance: float = .02
    dwell_seconds: float = 1.0
    timeout_seconds: float = 20.0
    instruction: str = "Move the tool above the red marker."
    camera_pos: tuple = (.35, 0., 1.2)
    camera_fovy: float = 55.
    image_width: int = 320
    image_height: int = 240
    timestep: float = 1. / 3000.
    control_hz: float = 100.
    camera_hz: float = 30.
    yaw_kp: float = 180.
    yaw_kv: float = 18.
    bend_kp: float = 420.
    bend_kv: float = 38.
    yaw_force: float = 14.
    bend_force: float = 25.
    speed_limit: float = 1.0
    accel_limit: float = 2.0
    home: tuple = (0., 0., -.9)
    workspace_radius: tuple = (.55, .68)
    workspace_angle: tuple = (-.65, .65)
    red_threshold: int = 110
    min_red_pixels: int = 30
    dls_damping: float = .02
    max_camera_step: float = .09
    probes: tuple = (
        Case("probe_center", (.60, 0.)),
        Case("probe_left", (.58*math.cos(-.50), .58*math.sin(-.50))),
        Case("probe_right", (.62*math.cos(.45), .62*math.sin(.45))),
        Case("probe_mid_neg", (.59*math.cos(-.30), .59*math.sin(-.30))),
    )
    shift_cases: tuple = (
        ShiftCase("shift_center_to_left", (.60, 0.),
                  (.58*math.cos(-.50), .58*math.sin(-.50))),
        ShiftCase("shift_right_to_center", (.62*math.cos(.45), .62*math.sin(.45)),
                  (.60, 0.)),
    )

    @property
    def control_decimation(self):
        return _exact_decimation(self.control_hz, self.timestep)

    @property
    def camera_decimation(self):
        return _exact_decimation(self.camera_hz, self.timestep)

    @property
    def logical_names(self):
        return ("base_yaw", "shoulder_bend", "elbow_bend")

    @property
    def internal_names(self):
        return ("yaw", "shoulder_a", "shoulder_b", "elbow_a", "elbow_b")

    @property
    def actuator_names(self):
        return ("yaw", "shoulder_bend", "elbow_bend")

    @property
    def marker_xy(self):
        return self.probes[0].marker_xy


def _exact_decimation(rate, dt):
    decimation = round(1. / (rate * dt))
    if decimation < 1 or abs(decimation * rate * dt - 1.) > 1e-12:
        raise ValueError(f"rate {rate} is not an exact integer tick rate at dt={dt}")
    return decimation


SCENARIO = Scenario()


def policy_config(cfg=SCENARIO):
    """Sanitized constants for the policy: no probe or marker-shift targets."""
    return replace(cfg, probes=(), shift_cases=())


def numbers(values):
    return " ".join(f"{v:.12g}" for v in values)


def _rolling_joints(cfg):
    return tuple(base_model.RollingJoint(j.name, j.proximal_radius, j.distal_radius,
                                         j.bend_limit, j.link_length, j.link_mass)
                 for j in cfg.joints)


def _hub_inertia(cfg):
    r = cfg.yaw_hub_radius
    h = 2. * cfg.yaw_hub_half_height
    m = cfg.yaw_hub_mass
    izz = .5 * m * r * r
    ixx = m * (3. * r * r + h * h) / 12.
    return (ixx, ixx, izz)


def _sphere_inertia(cfg):
    i = .4 * cfg.tool_mass * cfg.tool_radius * cfg.tool_radius
    return (i, i, i)


def _world_body(root, cfg):
    world = root.find("worldbody")
    base = world.find("./body[@name='base_link']")
    if base is None:
        raise ValueError("base model has no base_link body")
    world.remove(base)
    for child in list(world):
        world.remove(child)
    ET.SubElement(world, "light", name="key", pos="0 0 2.5", dir="0 0 -1",
                  directional="true", diffuse=".9 .9 .9")
    ET.SubElement(world, "geom", name="tabletop", type="plane", size="2 2 .01",
                  pos="0 0 0", rgba=".18 .20 .23 1", friction=".9 .01 .001")
    ET.SubElement(world, "geom", name="task_marker", type="cylinder",
                  pos=numbers((cfg.marker_xy[0], cfg.marker_xy[1], cfg.marker_half_height)),
                  size=numbers((cfg.marker_radius, cfg.marker_half_height)),
                  material="task_marker_red", contype="0", conaffinity="0",
                  friction="0 0 0")
    ET.SubElement(world, "camera", name="overhead", pos=numbers(cfg.camera_pos),
                  xyaxes="1 0 0 0 1 0", fovy=str(cfg.camera_fovy))
    _add_yaw_hub(world, cfg, base)
    return world


def _add_yaw_hub(world, cfg, base):
    hub = ET.SubElement(world, "body", name="yaw_hub",
                        pos=numbers((0., 0., cfg.shoulder_height)))
    ixx, iyy, izz = _hub_inertia(cfg)
    ET.SubElement(hub, "inertial", pos="0 0 0", mass=numbers((cfg.yaw_hub_mass,)),
                  diaginertia=numbers((ixx, iyy, izz)))
    ET.SubElement(hub, "joint", name="yaw", type="hinge", axis="0 0 1",
                  range=numbers((-cfg.yaw_limit, cfg.yaw_limit)),
                  damping="0.05", armature="0.001")
    ET.SubElement(hub, "geom", name="yaw_hub_geom", type="cylinder",
                  size=numbers((cfg.yaw_hub_radius, cfg.yaw_hub_half_height)),
                  rgba=".30 .33 .38 1", mass="0")
    base.set("pos", "0 0 0")
    base.set("quat", numbers((math.sqrt(.5), math.sqrt(.5), 0., 0.)))
    hub.append(base)
    return hub


def _add_tool_payload(cfg, root):
    elbow = root.find(".//body[@name='elbow_link']")
    if elbow is None:
        raise ValueError("base model has no elbow_link body")
    old_site = elbow.find("./site[@name='tool']")
    if old_site is not None:
        elbow.remove(old_site)
    pos = numbers((cfg.joints[-1].link_length, 0., cfg.tool_lateral_offset))
    tool = ET.SubElement(elbow, "body", name="tool_payload", pos=pos)
    ixx, iyy, izz = _sphere_inertia(cfg)
    ET.SubElement(tool, "inertial", pos="0 0 0", mass=numbers((cfg.tool_mass,)),
                  diaginertia=numbers((ixx, iyy, izz)))
    ET.SubElement(tool, "geom", name="tool_payload_geom", type="sphere",
                  size=numbers((cfg.tool_radius,)), rgba=".10 .75 .85 1",
                  mass="0", friction=".6 .01 .001")
    ET.SubElement(tool, "site", name="tool", size=".018", rgba=".10 .75 .85 1")
    return tool


def _add_controls(root, cfg):
    eq = root.find("equality")
    if eq is None:
        ET.SubElement(root, "equality")
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    for body1, body2 in (("yaw_hub", "base_link"), ("elbow_link", "tool_payload")):
        ET.SubElement(contact, "exclude", body1=body1, body2=body2)
    actuators = root.find("actuator")
    if actuators is None:
        actuators = ET.SubElement(root, "actuator")
    yaw = ET.Element("position", name="yaw_target", joint="yaw",
                     kp=numbers((cfg.yaw_kp,)), kv=numbers((cfg.yaw_kv,)),
                     ctrlrange=numbers((-cfg.yaw_limit, cfg.yaw_limit)),
                     forcerange=numbers((-cfg.yaw_force, cfg.yaw_force)))
    actuators.insert(0, yaw)
    for joint in cfg.joints:
        element = actuators.find(f"./position[@name='{joint.name}_target']")
        if element is None:
            raise ValueError(f"base model has no {joint.name}_target actuator")
        element.set("kp", numbers((cfg.bend_kp,)))
        element.set("kv", numbers((cfg.bend_kv,)))
        element.set("forcerange", numbers((-cfg.bend_force, cfg.bend_force)))


def validate_scenario(cfg=SCENARIO):
    def finite_positive(value, label):
        if not np.isfinite(value) or value <= 0.:
            raise ValueError(f"{label} must be finite and positive")

    if len(cfg.joints) != 2:
        raise ValueError("Expected two rolling pairs")
    names = [joint.name for joint in cfg.joints]
    if names != ["shoulder", "elbow"]:
        raise ValueError("Expected shoulder and elbow rolling pairs")
    for joint in cfg.joints:
        for value, label in ((joint.proximal_radius, "proximal radius"),
                             (joint.distal_radius, "distal radius"),
                             (joint.bend_limit, "bend limit"),
                             (joint.link_length, "link length"),
                             (joint.link_mass, "link mass")):
            finite_positive(value, f"{joint.name} {label}")
        if abs(joint.proximal_radius - joint.distal_radius) > 1e-12:
            raise ValueError("Selected mechanism requires equal rolling radii")
        if abs(joint.spacing - 2. * joint.proximal_radius) > 1e-12:
            raise ValueError("Rolling center spacing must be 2r")
    for value, label in ((cfg.carrier_mass, "carrier mass"),
                         (cfg.yaw_hub_mass, "yaw hub mass"),
                         (cfg.yaw_hub_radius, "yaw hub radius"),
                         (cfg.yaw_hub_half_height, "yaw hub half height"),
                         (cfg.tool_mass, "tool mass"),
                         (cfg.tool_radius, "tool radius"),
                         (cfg.yaw_limit, "yaw limit"),
                         (cfg.marker_radius, "marker radius"),
                         (cfg.goal_height, "goal height"),
                         (cfg.success_tolerance, "success tolerance"),
                         (cfg.dwell_seconds, "dwell time"),
                         (cfg.timeout_seconds, "timeout"),
                         (cfg.timestep, "timestep"),
                         (cfg.control_hz, "control rate"),
                         (cfg.camera_hz, "camera rate"),
                         (cfg.yaw_kp, "yaw gain"),
                         (cfg.yaw_kv, "yaw damping"),
                         (cfg.bend_kp, "bend gain"),
                         (cfg.bend_kv, "bend damping"),
                         (cfg.yaw_force, "yaw force cap"),
                         (cfg.bend_force, "bend force cap"),
                         (cfg.speed_limit, "speed limit"),
                         (cfg.accel_limit, "acceleration limit"),
                         (cfg.dls_damping, "DLS damping"),
                         (cfg.max_camera_step, "camera step limit"),
                         (cfg.marker_half_height, "marker half height")):
        finite_positive(value, label)
    if not np.isfinite(cfg.tool_lateral_offset):
        raise ValueError("Tool offset must be finite")
    if cfg.red_threshold < 0 or cfg.min_red_pixels < 1:
        raise ValueError("Red threshold and minimum pixel count are invalid")
    if not 0. < cfg.camera_fovy < 180.:
        raise ValueError("Camera fovy must be in (0, 180) degrees")
    if cfg.image_width < 1 or cfg.image_height < 1:
        raise ValueError("Image dimensions must be positive")
    if cfg.workspace_radius[0] <= 0. or cfg.workspace_radius[0] >= cfg.workspace_radius[1]:
        raise ValueError("Workspace radius range must be positive and increasing")
    if (not np.isfinite(cfg.workspace_angle).all()
            or cfg.workspace_angle[0] >= cfg.workspace_angle[1]):
        raise ValueError("Workspace angle range must be finite and increasing")
    limits = np.array([cfg.yaw_limit, cfg.joints[0].bend_limit,
                       cfg.joints[1].bend_limit])
    home = np.asarray(cfg.home, dtype=float)
    if home.shape != (3,) or not np.isfinite(home).all() or np.any(np.abs(home) > limits):
        raise ValueError("Home pose must be three finite coordinates within limits")
    for case in cfg.probes:
        if not marker_in_workspace(case.marker_xy, cfg):
            raise ValueError(f"Probe {case.name} is outside the declared workspace")
    for case in cfg.shift_cases:
        if not marker_in_workspace(case.start_xy, cfg):
            raise ValueError(f"Shift start {case.name} is outside the declared workspace")
        if not marker_in_workspace(case.shift_xy, cfg):
            raise ValueError(f"Shift target {case.name} is outside the declared workspace")
    return {"control_decimation": cfg.control_decimation,
            "camera_decimation": cfg.camera_decimation,
            "home_within_limits": True,
            "probes_and_shifts_in_workspace": True}


def mjcf(cfg=SCENARIO):
    validate_scenario(cfg)
    root = base_model.mjcf(_rolling_joints(cfg))
    root.set("model", cfg.name)
    option = root.find("option")
    option.set("timestep", numbers((cfg.timestep,)))
    option.set("gravity", "0 0 -9.81")
    option.set("integrator", "implicitfast")
    option.set("iterations", "100")
    option.set("tolerance", "1e-10")
    # A class-level rgba silently overrides an explicit material, so drop the
    # inherited default color. Every inherited geom already declares its color.
    default_geom = root.find("./default/geom")
    if default_geom is not None:
        default_geom.attrib.pop("rgba", None)
    asset = ET.Element("asset")
    ET.SubElement(asset, "material", name="task_marker_red", rgba="1 0 0 1",
                  emission="0", specular="0", shininess="0")
    root.insert(list(root).index(root.find("default")), asset)
    # The inherited orange elbow produces dark facets that pass a red-hue
    # test. Recolor it so the tabletop marker is the only red object.
    for name in ("elbow_distal_wheel", "elbow_beam"):
        geom = root.find(f".//geom[@name='{name}']")
        if geom is not None:
            geom.set("rgba", ".20 .55 .50 1")
    _world_body(root, cfg)
    _add_tool_payload(cfg, root)
    _add_controls(root, cfg)
    return root


def urdf(cfg=SCENARIO):
    validate_scenario(cfg)
    root = base_model.urdf(_rolling_joints(cfg))
    root.set("name", cfg.name)
    extension = root.find("mujoco")
    if extension is None:
        extension = ET.SubElement(root, "mujoco")
    compiler = extension.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(extension, "compiler")
    compiler.set("discardvisual", "false")
    compiler.set("fusestatic", "false")

    base = root.find("./link[@name='base_link']")
    if base is None:
        raise ValueError("base URDF has no base_link")
    for element in base.findall("./visual/origin") + base.findall("./collision/origin"):
        element.set("xyz", "0 0 0")
    shoulder_joint = root.find("./joint[@name='shoulder_a']")
    if shoulder_joint is None:
        raise ValueError("base URDF has no shoulder_a joint")
    shoulder_joint.find("origin").set("xyz", "0 0 0")
    # The inherited visual wheel would otherwise gain importer-inferred mass.
    _urdf_inertia(base, 0., (0., 0., 0.), (0., 0., 0.))
    zero_inertia = base[-1]
    base.remove(zero_inertia)
    base.insert(0, zero_inertia)

    world_mount = ET.Element("link", name="world_mount")
    root.append(world_mount)
    hub = ET.Element("link", name="yaw_hub")
    ixx, iyy, izz = _hub_inertia(cfg)
    _urdf_inertia(hub, cfg.yaw_hub_mass, (0., 0., 0.), (ixx, iyy, izz))
    _urdf_shape(hub, "cylinder", {"radius": numbers((cfg.yaw_hub_radius,)),
                                  "length": numbers((2. * cfg.yaw_hub_half_height,))},
                (0., 0., 0.), (.30, .33, .38, 1.))
    root.append(hub)
    yaw_joint = ET.SubElement(root, "joint", name="yaw", type="revolute")
    ET.SubElement(yaw_joint, "parent", link="world_mount")
    ET.SubElement(yaw_joint, "child", link="yaw_hub")
    ET.SubElement(yaw_joint, "origin", xyz=numbers((0., 0., cfg.shoulder_height)), rpy="0 0 0")
    ET.SubElement(yaw_joint, "axis", xyz="0 0 1")
    ET.SubElement(yaw_joint, "limit", lower=numbers((-cfg.yaw_limit,)),
                  upper=numbers((cfg.yaw_limit,)), effort=numbers((cfg.yaw_force,)), velocity="2")
    mount = ET.SubElement(root, "joint", name="yaw_mount", type="fixed")
    ET.SubElement(mount, "parent", link="yaw_hub")
    ET.SubElement(mount, "child", link="base_link")
    ET.SubElement(mount, "origin", xyz="0 0 0", rpy=numbers((math.pi / 2., 0., 0.)))

    root.find("./joint[@name='tool_fixed']/origin").set(
        "xyz", numbers((cfg.joints[-1].link_length, 0., cfg.tool_lateral_offset)))
    for color in root.findall("./link[@name='elbow_link']/visual/material/color"):
        color.set("rgba", ".20 .55 .50 1")
    tool = root.find("./link[@name='tool']")
    if tool is None:
        tool = ET.SubElement(root, "link", name="tool")
    for child in list(tool):
        tool.remove(child)
    ixx, iyy, izz = _sphere_inertia(cfg)
    _urdf_inertia(tool, cfg.tool_mass, (0., 0., 0.), (ixx, iyy, izz))
    _urdf_shape(tool, "sphere", {"radius": numbers((cfg.tool_radius,))},
                (0., 0., 0.), (.10, .75, .85, 1.))
    return root


def _urdf_inertia(link, mass, com, diagonal):
    node = ET.SubElement(link, "inertial")
    ET.SubElement(node, "mass", value=numbers((mass,)))
    ET.SubElement(node, "origin", xyz=numbers(com))
    ET.SubElement(node, "inertia", ixx=numbers((diagonal[0],)), iyy=numbers((diagonal[1],)),
                  izz=numbers((diagonal[2],)), ixy="0", ixz="0", iyz="0")


def _urdf_shape(link, kind, attrs, position, color):
    for tag in ("visual", "collision"):
        node = ET.SubElement(link, tag)
        ET.SubElement(node, "origin", xyz=numbers(position))
        ET.SubElement(ET.SubElement(node, "geometry"), kind, attrs)
        if tag == "visual":
            ET.SubElement(ET.SubElement(node, "material", name=""), "color", rgba=numbers(color))


def write_xml(path, root):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def generated_xml(cfg=SCENARIO):
    return ET.tostring(mjcf(cfg), encoding="unicode")


def generated_urdf_xml(cfg=SCENARIO):
    return ET.tostring(urdf(cfg), encoding="unicode")


def write_generated(directory, cfg=SCENARIO):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    write_xml(directory / "arm.xml", mjcf(cfg))
    write_xml(directory / "arm.urdf", urdf(cfg))
    (directory / "manifest.json").write_text(json.dumps(effective_manifest(cfg), indent=2) + "\n")
    return directory


def _joint_dict(j, cfg):
    return {
        "name": j.name,
        "proximal_radius_m": j.proximal_radius,
        "distal_radius_m": j.distal_radius,
        "center_spacing_m": j.spacing,
        "radius_ratio": j.ratio,
        "bend_limit_rad": j.bend_limit,
        "link_length_m": j.link_length,
        "link_mass_kg": j.link_mass,
        "carrier_mass_kg": cfg.carrier_mass,
    }


def effective_manifest(cfg=SCENARIO):
    return {
        "model": cfg.name,
        "model_revision": cfg.revision,
        "assumptions_only": True,
        "not_calibrated_to_hardware": True,
        "units": "meters, kilograms, radians, seconds",
        "gravity_m_s2": [0., 0., -9.81],
        "mount": {
            "shoulder_height_m": cfg.shoulder_height,
            "pre_yaw_mount": "Rx(+pi/2)",
            "yaw_axis_world": [0., 0., 1.],
            "local_plane": "old local XY maps to world XZ at zero yaw",
            "local_positive_z_maps_to_world": [0., -1., 0.],
        },
        "joints": [_joint_dict(j, cfg) for j in cfg.joints],
        "action_semantics": "absolute logical joint targets in radians",
        "logical_coordinates": list(cfg.logical_names),
        "internal_coordinates": list(cfg.internal_names),
        "internal_to_logical": observer(cfg).tolist(),
        "actuator_order": list(cfg.actuator_names),
        "masses_kg": {
            "yaw_hub": cfg.yaw_hub_mass,
            "carriers_each": cfg.carrier_mass,
            "links": [j.link_mass for j in cfg.joints],
            "tool_payload": cfg.tool_mass,
            "total": cfg.yaw_hub_mass + len(cfg.joints) * cfg.carrier_mass +
                     sum(j.link_mass for j in cfg.joints) + cfg.tool_mass,
        },
        "inertia_assumptions": {
            "yaw_hub_cylinder_diagonal_kg_m2": list(_hub_inertia(cfg)),
            "tool_sphere_diagonal_kg_m2": list(_sphere_inertia(cfg)),
            "inherited_declarations": "carrier and link COM/inertias come from rolling_contact_arm public model",
        },
        "tool": {"mass_kg": cfg.tool_mass, "radius_m": cfg.tool_radius,
                 "local_offset_m": [cfg.joints[-1].link_length, 0., cfg.tool_lateral_offset], "color": "cyan"},
        "table": {"top_z_m": cfg.table_z, "marker_radius_m": cfg.marker_radius,
                  "marker_is_noncolliding": True, "marker_color": "red"},
        "task": {
            "instruction": cfg.instruction,
            "goal": "marker XY at table_z + goal_height",
            "goal_height_m": cfg.goal_height,
            "success_tolerance_m": cfg.success_tolerance,
            "dwell_seconds": cfg.dwell_seconds,
            "timeout_seconds": cfg.timeout_seconds,
            "counts_all_attempts": True,
        },
        "camera": {"name": "overhead", "pos_m": list(cfg.camera_pos),
                   "xyaxes": [1, 0, 0, 0, 1, 0], "fovy_deg": cfg.camera_fovy,
                   "width": cfg.image_width, "height": cfg.image_height,
                   "calibration": "known fixed camera; policy receives RGB only"},
        "timing": {"timestep_s": cfg.timestep, "control_hz": cfg.control_hz,
                   "control_decimation": cfg.control_decimation,
                   "camera_hz": cfg.camera_hz,
                   "camera_decimation": cfg.camera_decimation,
                   "tick_based_time": True},
        "limits": {
            "yaw_rad": cfg.yaw_limit,
            "bend_rad": [j.bend_limit for j in cfg.joints],
            "speed_rad_s": cfg.speed_limit,
            "accel_rad_s2": cfg.accel_limit,
            "yaw_force_nm": cfg.yaw_force,
            "bend_force_nm": cfg.bend_force,
            "dls_damping": cfg.dls_damping,
            "max_camera_step_rad": cfg.max_camera_step,
        },
        "home_logical_rad": list(cfg.home),
        "workspace": {"radius_m": list(cfg.workspace_radius),
                      "angle_rad": list(cfg.workspace_angle),
                      "goal_height_m": cfg.goal_height},
        "probes": [{"name": c.name, "marker_xy_m": list(c.marker_xy)} for c in cfg.probes],
        "shift_cases": [{"name": c.name, "start_xy_m": list(c.start_xy),
                         "shift_xy_m": list(c.shift_xy), "shift_time_s": c.shift_time}
                        for c in cfg.shift_cases],
        "mechanism": "Equal-radius ideal external circular rolling pairs; tendon bend=a+b; no slip",
        "actuation": "Assumed logical tendon torque, not the user's actual transmission",
        "not_modeled": ["surface slip", "tendon compliance", "backlash", "motor dynamics",
                        "actual transmission", "user final CAD", "physical calibration"],
    }


def logical_to_internal(q, cfg=SCENARIO):
    q = np.asarray(q, dtype=float)
    if q.shape != (3,) or not np.isfinite(q).all():
        raise ValueError("Expected three finite logical coordinates")
    if abs(q[0]) > cfg.yaw_limit + 1e-12:
        raise ValueError("Yaw target exceeds limit")
    for value, joint in zip(q[1:], cfg.joints):
        if abs(value) > joint.bend_limit + 1e-12:
            raise ValueError("Logical bend exceeds limit")
    result = [q[0]]
    for value, joint in zip(q[1:], cfg.joints):
        result.extend((value / (1. + joint.ratio), value * joint.ratio / (1. + joint.ratio)))
    return np.asarray(result)


def internal_to_logical(q, cfg=SCENARIO):
    q = np.asarray(q, dtype=float)
    if q.shape != (5,) or not np.isfinite(q).all():
        raise ValueError("Expected five finite internal coordinates")
    return np.array([q[0], q[1] + q[2], q[3] + q[4]])


def mapping(cfg=SCENARIO):
    g = np.zeros((5, 3))
    g[0, 0] = 1.
    for i, joint in enumerate(cfg.joints):
        g[1 + 2*i, 1 + i] = 1. / (1. + joint.ratio)
        g[2 + 2*i, 1 + i] = joint.ratio / (1. + joint.ratio)
    return g


def observer(cfg=SCENARIO):
    s = np.zeros((3, 5))
    s[0, 0] = 1.
    s[1, 1:3] = 1.
    s[2, 3:5] = 1.
    return s


def actuator_moment(cfg=SCENARIO):
    return observer(cfg)


def _planar(q, cfg):
    q = np.asarray(q, dtype=float)
    yaw, shoulder, elbow = q
    s, e = cfg.joints
    ks = 1. / (1. + s.ratio)
    ke = 1. / (1. + e.ratio)
    a_s = ks * shoulder
    a_e = ke * elbow
    X = (s.spacing * np.cos(a_s) + s.link_length * np.cos(shoulder) +
         e.spacing * np.cos(shoulder + a_e) + e.link_length * np.cos(shoulder + elbow))
    Z = (s.spacing * np.sin(a_s) + s.link_length * np.sin(shoulder) +
         e.spacing * np.sin(shoulder + a_e) + e.link_length * np.sin(shoulder + elbow))
    return X, Z, a_s, a_e, ks, ke


def analytic_fk(q, cfg=SCENARIO):
    q = np.asarray(q, dtype=float)
    if q.shape != (3,) or not np.isfinite(q).all():
        raise ValueError("Expected three finite logical coordinates")
    X, Z, _, _, _, _ = _planar(q, cfg)
    yaw = q[0]
    c, s = np.cos(yaw), np.sin(yaw)
    offset = cfg.tool_lateral_offset
    return np.array([c * X + s * offset, s * X - c * offset, cfg.shoulder_height + Z])


def analytic_links(q, cfg=SCENARIO):
    """World positions of the shoulder and elbow wheel-link origins."""
    q = np.asarray(q, dtype=float)
    if q.shape != (3,) or not np.isfinite(q).all():
        raise ValueError("Expected three finite logical coordinates")
    yaw, shoulder, elbow = q
    s, e = cfg.joints
    a_s = shoulder / (1. + s.ratio)
    a_e = elbow / (1. + e.ratio)
    shoulder_x = s.spacing * np.cos(a_s)
    shoulder_z = s.spacing * np.sin(a_s)
    elbow_x = (shoulder_x + s.link_length * np.cos(shoulder)
               + e.spacing * np.cos(shoulder + a_e))
    elbow_z = (shoulder_z + s.link_length * np.sin(shoulder)
               + e.spacing * np.sin(shoulder + a_e))
    c, sn = np.cos(yaw), np.sin(yaw)
    return (np.array([c * shoulder_x, sn * shoulder_x, cfg.shoulder_height + shoulder_z]),
            np.array([c * elbow_x, sn * elbow_x, cfg.shoulder_height + elbow_z]))


def analytic_jacobian(q, cfg=SCENARIO):
    q = np.asarray(q, dtype=float)
    if q.shape != (3,) or not np.isfinite(q).all():
        raise ValueError("Expected three finite logical coordinates")
    yaw, shoulder, elbow = q
    s, e = cfg.joints
    ks = 1. / (1. + s.ratio)
    ke = 1. / (1. + e.ratio)
    a_s = ks * shoulder
    a_e = ke * elbow
    X, Z, _, _, _, _ = _planar(q, cfg)
    dX_ds = (-s.spacing * ks * np.sin(a_s) - s.link_length * np.sin(shoulder)
             - e.spacing * np.sin(shoulder + a_e) - e.link_length * np.sin(shoulder + elbow))
    dZ_ds = (s.spacing * ks * np.cos(a_s) + s.link_length * np.cos(shoulder)
             + e.spacing * np.cos(shoulder + a_e) + e.link_length * np.cos(shoulder + elbow))
    dX_de = (-e.spacing * ke * np.sin(shoulder + a_e)
             - e.link_length * np.sin(shoulder + elbow))
    dZ_de = (e.spacing * ke * np.cos(shoulder + a_e)
             + e.link_length * np.cos(shoulder + elbow))
    c, sn = np.cos(yaw), np.sin(yaw)
    offset = cfg.tool_lateral_offset
    jac = np.zeros((3, 3))
    jac[:, 0] = [-sn * X + c * offset, c * X + sn * offset, 0.]
    jac[:, 1] = [c * dX_ds, sn * dX_ds, dZ_ds]
    jac[:, 2] = [c * dX_de, sn * dX_de, dZ_de]
    return jac


@dataclass(frozen=True)
class CameraCalibration:
    pos: tuple
    rotation: tuple
    fovy: float
    width: int
    height: int

    @property
    def tan_half_fovy(self):
        return math.tan(math.radians(self.fovy) / 2.)

    @property
    def aspect(self):
        return self.width / self.height


def camera_calibration(cfg=SCENARIO):
    # xyaxes=1 0 0 0 1 0 gives an identity camera frame that looks down world -Z.
    return CameraCalibration(tuple(cfg.camera_pos), ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)),
                             cfg.camera_fovy, cfg.image_width, cfg.image_height)


def project_points(points, cam):
    points = np.atleast_2d(np.asarray(points, dtype=float))
    R = np.asarray(cam.rotation)
    p = (points - np.asarray(cam.pos)) @ R
    valid = p[:, 2] < -1e-9
    u = np.full(len(p), np.nan)
    v = np.full(len(p), np.nan)
    depth = -p[:, 2]
    good = valid & (depth > 1e-12)
    if good.any():
        x = p[good, 0] / depth[good] / (cam.tan_half_fovy * cam.aspect)
        y = p[good, 1] / depth[good] / cam.tan_half_fovy
        u[good] = (x + 1.) * .5 * cam.width - .5
        v[good] = (1. - y) * .5 * cam.height - .5
    return u, v, valid


def unproject_to_plane(u, v, plane_z, cam):
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)
    R = np.asarray(cam.rotation)
    xn = (u + .5) / cam.width * 2. - 1.
    yn = 1. - (v + .5) / cam.height * 2.
    local = np.stack([xn * cam.tan_half_fovy * cam.aspect,
                      yn * cam.tan_half_fovy,
                      -np.ones_like(xn)], axis=-1)
    direction = local @ R.T
    dz = direction[..., 2]
    parallel = np.abs(dz) < 1e-12
    safe = np.where(parallel, 1., dz)
    t = (plane_z - cam.pos[2]) / safe
    valid = (~parallel) & (t > 0.)
    point = np.asarray(cam.pos) + direction * t[..., None]
    point[~valid] = np.nan
    return point, valid


def red_mask(rgb, cfg=SCENARIO):
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] < 3:
        raise ValueError("Expected an RGB image")
    r = rgb[..., 0].astype(np.int16)
    g = rgb[..., 1].astype(np.int16)
    b = rgb[..., 2].astype(np.int16)
    return (r >= cfg.red_threshold) & (g * 2 < r) & (b * 2 < r) & (r - g > 50) & (r - b > 50)


def marker_radius_px(cfg=SCENARIO, cam=None):
    cam = cam or camera_calibration(cfg)
    depth = cam.pos[2] - cfg.table_z
    return cfg.marker_radius * cam.height / (2. * depth * cam.tan_half_fovy)


def red_boundary(mask):
    padded = np.pad(mask, 1)
    interior = (padded[:-2, 1:-1] & padded[2:, 1:-1] &
                padded[1:-1, :-2] & padded[1:-1, 2:])
    return np.nonzero(mask & ~interior)


def red_component_union(mask, min_size=10):
    """Keep substantial red blobs and discard isolated highlight specks."""
    remaining = set(map(tuple, np.argwhere(mask)))
    kept = set()
    while remaining:
        seed = remaining.pop()
        component = {seed}
        stack = [seed]
        while stack:
            row, col = stack.pop()
            for neighbor in ((row + 1, col), (row - 1, col),
                             (row, col + 1), (row, col - 1)):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    stack.append(neighbor)
        if len(component) >= min_size:
            kept.update(component)
    out = np.zeros_like(mask, dtype=bool)
    if kept:
        rows, cols = zip(*kept)
        out[np.asarray(rows), np.asarray(cols)] = True
    return out


def red_center(rgb, cfg=SCENARIO, cam=None):
    """Fit the known projected marker radius to the visible red boundary.

    The arm and tool can occlude part of the marker.  A centroid is biased by
    that occlusion, while a fixed-radius fit uses the surviving outer arc.
    """
    cam = cam or camera_calibration(cfg)
    mask = red_component_union(red_mask(rgb, cfg))
    count = int(mask.sum())
    if count < cfg.min_red_pixels:
        return None, count, None
    rows, cols = np.nonzero(mask)
    xmin, xmax = float(cols.min()), float(cols.max())
    ymin, ymax = float(rows.min()), float(rows.max())
    cx = (xmin + xmax) / 2.
    cy = (ymin + ymax) / 2.
    by, bx = red_boundary(mask)
    if len(bx) < 8:
        return None, count, {"reason": "too few boundary pixels"}
    px = bx.astype(float)
    py = by.astype(float)
    radius = marker_radius_px(cfg, cam)
    kept = None
    for iteration in range(12):
        dx = px - cx
        dy = py - cy
        distance = np.hypot(dx, dy)
        tolerance = 2.5 if iteration < 4 else 1.5
        kept = np.abs(distance - radius) < tolerance
        if int(kept.sum()) < 8:
            return None, count, {"reason": "few boundary inliers",
                                 "radius_px": float(radius)}
        residual = distance[kept] - radius
        jac = np.stack([-dx[kept] / distance[kept], -dy[kept] / distance[kept]], axis=1)
        delta = np.linalg.lstsq(jac, -residual, rcond=None)[0]
        cx += float(delta[0])
        cy += float(delta[1])
        if np.linalg.norm(delta) < 1e-4:
            break
    dx = px - cx
    dy = py - cy
    distance = np.hypot(dx, dy)
    kept = np.abs(distance - radius) < 1.5
    if int(kept.sum()) < 8:
        return None, count, {"reason": "fit did not converge",
                             "radius_px": float(radius)}
    rms = float(np.sqrt(np.mean((distance[kept] - radius) ** 2)))
    if rms > 1.5 or not (xmin - 3. <= cx <= xmax + 3.) or not (ymin - 3. <= cy <= ymax + 3.):
        return None, count, {"reason": "fit residual", "rms_px": rms,
                             "radius_px": float(radius)}
    return (float(cy), float(cx)), count, {"radius_px": float(radius),
                                           "inliers": int(kept.sum()), "rms_px": rms}


def marker_in_workspace(xy, cfg=SCENARIO):
    x, y = float(xy[0]), float(xy[1])
    radius = math.hypot(x, y)
    angle = math.atan2(y, x)
    return (cfg.workspace_radius[0] <= radius <= cfg.workspace_radius[1]
            and cfg.workspace_angle[0] <= angle <= cfg.workspace_angle[1])
