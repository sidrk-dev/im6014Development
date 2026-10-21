"""Illustrative circular rolling joints, independent of the user's CAD assets."""

from dataclasses import asdict, dataclass
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent


@dataclass(frozen=True)
class RollingJoint:
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


JOINTS = (
    RollingJoint("shoulder", .05, .05, 1.2, .32, .7),
    RollingJoint("elbow", .04, .04, 1.4, .28, .4),
)
BASE = np.array([0., 0., .8])
TIMESTEP = 1 / 3000
COLORS = ((.13, .65, .91, 1.), (.96, .56, .18, 1.))


def numbers(values):
    return " ".join(f"{v:.12g}" for v in values)


def logical_to_internal(bends, joints=JOINTS):
    bends = np.asarray(bends, dtype=float)
    if bends.shape != (2,) or not np.isfinite(bends).all():
        raise ValueError("Expected two finite logical bend coordinates")
    if any(abs(q) > joint.bend_limit for q, joint in zip(bends, joints)):
        raise ValueError("Logical target exceeds sample joint limits")
    return np.array([v for q, j in zip(bends, joints)
                     for v in (q / (1 + j.ratio), q * j.ratio / (1 + j.ratio))])


def internal_to_logical(internal):
    internal = np.asarray(internal, dtype=float)
    if internal.shape != (4,) or not np.isfinite(internal).all():
        raise ValueError("Expected four finite internal hinge coordinates")
    return internal.reshape(2, 2).sum(axis=1)


def write_xml(path, root):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def mjcf(joints=JOINTS):
    root = ET.Element("mujoco", model="illustrative_rolling_contact_arm")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep=str(TIMESTEP), gravity="0 -9.81 0",
                  integrator="implicitfast", iterations="100", tolerance="1e-10")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="960", offheight="640")
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "joint", axis="0 0 1", damping="0.03", armature="0.0001")
    ET.SubElement(default, "geom", friction="0.6 0.01 0.001", rgba=".4 .5 .6 1")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 0 2.5", dir="0 0 -1", directional="true")
    ET.SubElement(world, "camera", name="inspection", pos=".28 0 2.2",
                  xyaxes="1 0 0 0 1 0", fovy="48")
    ET.SubElement(world, "geom", type="plane", size="2 2 .01", pos="0 0 .7",
                  rgba=".07 .09 .13 1", contype="0", conaffinity="0")
    parent = ET.SubElement(world, "body", name="base_link", pos=numbers(BASE))
    eq = ET.SubElement(root, "equality")
    tendons = ET.SubElement(root, "tendon")
    actuators = ET.SubElement(root, "actuator")
    contact = ET.SubElement(root, "contact")
    for i, j in enumerate(joints):
        previous_name = "base_link" if i == 0 else f"{joints[i-1].name}_link"
        pivot = (0, 0, 0) if i == 0 else (joints[i-1].link_length, 0, 0)
        ET.SubElement(parent, "geom", name=f"{j.name}_proximal_wheel", type="cylinder",
                      pos=numbers(pivot), size=numbers((j.proximal_radius, .012)),
                      rgba=".55 .61 .68 1", mass="0")
        carrier = ET.SubElement(parent, "body", name=f"{j.name}_carrier", pos=numbers(pivot))
        ET.SubElement(carrier, "inertial", pos=numbers((j.spacing/2, 0, -.03)),
                      mass=".025", diaginertia=".00001 .00004 .00004")
        ET.SubElement(carrier, "joint", name=f"{j.name}_a", type="hinge",
                      range=numbers((-j.bend_limit/(1+j.ratio), j.bend_limit/(1+j.ratio))))
        ET.SubElement(carrier, "geom", type="capsule",
                      fromto=numbers((0, 0, -.035, j.spacing, 0, -.035)), size=".008",
                      rgba=".8 .8 .82 1", contype="0", conaffinity="0", mass="0")
        link = ET.SubElement(carrier, "body", name=f"{j.name}_link", pos=numbers((j.spacing, 0, 0)))
        ET.SubElement(link, "joint", name=f"{j.name}_b", type="hinge",
                      range=numbers((-j.bend_limit*j.ratio/(1+j.ratio), j.bend_limit*j.ratio/(1+j.ratio))))
        ET.SubElement(link, "inertial", pos=numbers((j.link_length*.42, 0, 0)),
                      mass=str(j.link_mass), diaginertia=numbers((.0004, .006 if i == 0 else .003,
                                                                                .006 if i == 0 else .003)))
        ET.SubElement(link, "geom", name=f"{j.name}_distal_wheel", type="cylinder",
                      size=numbers((j.distal_radius, .012)), rgba=numbers(COLORS[i]), mass="0")
        ET.SubElement(link, "geom", name=f"{j.name}_beam", type="capsule",
                      fromto=numbers((0, 0, .025, j.link_length, 0, .025)), size=".016",
                      rgba=numbers(COLORS[i]), mass="0")
        ET.SubElement(link, "geom", type="capsule", fromto=numbers((0, 0, .014, j.distal_radius*.85, 0, .014)),
                      size=".003", rgba="1 1 1 1", mass="0", contype="0", conaffinity="0")
        ET.SubElement(link, "site", name=f"{j.name}_center", size=".004")
        ET.SubElement(eq, "joint", name=f"{j.name}_rolling", joint1=f"{j.name}_b",
                      joint2=f"{j.name}_a", polycoef=numbers((0, j.ratio, 0, 0, 0)),
                      solref=".002 1", solimp=".9999 .9999 .001")
        tendon = ET.SubElement(tendons, "fixed", name=f"{j.name}_bend")
        for suffix in ("a", "b"):
            ET.SubElement(tendon, "joint", joint=f"{j.name}_{suffix}", coef="1")
        ET.SubElement(actuators, "position", name=f"{j.name}_target", tendon=f"{j.name}_bend",
                      kp="120", kv="12", ctrlrange=numbers((-j.bend_limit, j.bend_limit)),
                      forcerange="-25 25")
        ET.SubElement(contact, "exclude", body1=previous_name, body2=f"{j.name}_link")
        parent = link
    ET.SubElement(parent, "site", name="tool", pos=numbers((joints[-1].link_length, 0, .025)),
                  size=".018", rgba=".9 .2 .3 1")
    return root


def urdf(joints=JOINTS):
    root = ET.Element("robot", name="illustrative_rolling_contact_arm")
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(extension, "compiler", discardvisual="false", fusestatic="false")
    parent = ET.SubElement(root, "link", name="base_link")
    for i, j in enumerate(joints):
        previous = "base_link" if i == 0 else f"{joints[i-1].name}_link"
        pivot = BASE if i == 0 else (joints[i-1].link_length, 0, 0)
        shape(parent, "cylinder", {"radius":str(j.proximal_radius), "length":".024"},
              pivot, (.55, .61, .68, 1))
        carrier = ET.SubElement(root, "link", name=f"{j.name}_carrier")
        inertia(carrier, .025, (j.spacing/2, 0, -.03), (.00001, .00004, .00004))
        shape(carrier, "box", {"size":numbers((j.spacing, .016, .016))},
              (j.spacing/2, 0, -.035), (.8, .8, .82, 1), collision=False)
        link = ET.SubElement(root, "link", name=f"{j.name}_link")
        inertia(link, j.link_mass, (j.link_length*.42, 0, 0),
                (.0004, .006 if i == 0 else .003, .006 if i == 0 else .003))
        shape(link, "cylinder", {"radius":str(j.distal_radius), "length":".024"},
              (0, 0, 0), COLORS[i])
        shape(link, "box", {"size":numbers((j.link_length, .032, .032))},
              (j.link_length/2, 0, .025), COLORS[i])
        for suffix, origin, predecessor, child, factor in (
            ("a", pivot, previous, f"{j.name}_carrier", 1/(1+j.ratio)),
            ("b", (j.spacing, 0, 0), f"{j.name}_carrier", f"{j.name}_link", j.ratio/(1+j.ratio)),
        ):
            joint = ET.SubElement(root, "joint", name=f"{j.name}_{suffix}", type="revolute")
            ET.SubElement(joint, "parent", link=predecessor)
            ET.SubElement(joint, "child", link=child)
            ET.SubElement(joint, "origin", xyz=numbers(origin), rpy="0 0 0")
            ET.SubElement(joint, "axis", xyz="0 0 1")
            ET.SubElement(joint, "limit", lower=str(-j.bend_limit*factor), upper=str(j.bend_limit*factor),
                          effort="25", velocity="2")
            if suffix == "b":
                ET.SubElement(joint, "mimic", joint=f"{j.name}_a", multiplier=str(j.ratio), offset="0")
        parent = link
    ET.SubElement(root, "link", name="tool")
    tool = ET.SubElement(root, "joint", name="tool_fixed", type="fixed")
    ET.SubElement(tool, "parent", link=f"{joints[-1].name}_link")
    ET.SubElement(tool, "child", link="tool")
    ET.SubElement(tool, "origin", xyz=numbers((joints[-1].link_length, 0, .025)))
    return root


def inertia(link, mass, com, diagonal):
    node = ET.SubElement(link, "inertial")
    ET.SubElement(node, "mass", value=str(mass))
    ET.SubElement(node, "origin", xyz=numbers(com))
    ET.SubElement(node, "inertia", ixx=str(diagonal[0]), iyy=str(diagonal[1]), izz=str(diagonal[2]),
                  ixy="0", ixz="0", iyz="0")


def shape(link, kind, attrs, position, color, collision=True):
    for tag in (("visual", "collision") if collision else ("visual",)):
        node = ET.SubElement(link, tag)
        ET.SubElement(node, "origin", xyz=numbers(position))
        ET.SubElement(ET.SubElement(node, "geometry"), kind, attrs)
        if tag == "visual":
            ET.SubElement(ET.SubElement(node, "material", name=""), "color", rgba=numbers(color))


def generate():
    write_xml(HERE / "arm.xml", mjcf())
    write_xml(HERE / "arm.urdf", urdf())
    manifest = {"illustrative_only": True, "model_revision": "circular-sample-v1",
                "plane": "XY", "joint_axis": "+Z", "gravity": [0, -9.81, 0],
                "base": BASE.tolist(), "joints": [asdict(j) for j in JOINTS],
                "logical_coordinates": [f"{j.name}_bend" for j in JOINTS],
                "internal_coordinates": [f"{j.name}_{s}" for j in JOINTS for s in ("a", "b")],
                "carrier_mass_kg_each": .025, "units": "meters, kilograms, radians, seconds",
                "mechanism": "Ideal external circular rolling, constrained center spacing, zero slip",
                "not_modeled": ["surface slip", "tendon compliance", "backlash", "actual motor transmission", "user final CAD"]}
    (HERE / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    generate()
