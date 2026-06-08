#!/usr/bin/env python3
"""
URDF to MuJoCo XML Converter

Converts a URDF file describing a simple kinematic tree into a MuJoCo XML
(MJCF) file. Handles:
  - Links with inertial, visual, and collision elements
  - Mesh geometries (STL) with scale
  - Primitive geometries (box, cylinder, sphere)
  - Materials (color rgba)
  - Revolute, continuous, prismatic, and fixed joints
  - Kinematic tree nesting (parent-child relationships)
  - Closed kinematic chains via virtual links → equality/connect constraints

Usage:
    python urdf_to_xml.py <input.urdf> [output.xml] [--meshdir DIR]
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from xml.dom import minidom
from dataclasses import dataclass, field
from typing import Optional


# ---------------------------------------------------------------------------
# Data classes for intermediate representation
# ---------------------------------------------------------------------------

@dataclass
class Inertial:
    origin_xyz: str = "0 0 0"
    origin_rpy: str = "0 0 0"
    mass: float = 0.0
    ixx: float = 0.0
    iyy: float = 0.0
    izz: float = 0.0
    ixy: float = 0.0
    iyz: float = 0.0
    ixz: float = 0.0


@dataclass
class Geometry:
    geom_type: str = ""        # "mesh", "box", "cylinder", "sphere"
    mesh_filename: str = ""
    mesh_scale: str = ""
    box_size: str = ""         # URDF full extents → MuJoCo half-extents
    cylinder_radius: float = 0.0
    cylinder_length: float = 0.0
    sphere_radius: float = 0.0


@dataclass
class Visual:
    origin_xyz: str = "0 0 0"
    origin_rpy: str = "0 0 0"
    geometry: Optional[Geometry] = None
    material_name: str = ""


@dataclass
class Collision:
    origin_xyz: str = "0 0 0"
    origin_rpy: str = "0 0 0"
    geometry: Optional[Geometry] = None


@dataclass
class Link:
    name: str = ""
    inertial: Optional[Inertial] = None
    visuals: list = field(default_factory=list)       # List[Visual]
    collisions: list = field(default_factory=list)     # List[Collision]


@dataclass
class Joint:
    name: str = ""
    joint_type: str = ""       # revolute, continuous, prismatic, fixed
    origin_xyz: str = "0 0 0"
    origin_rpy: str = "0 0 0"
    parent: str = ""
    child: str = ""
    axis: str = "1 0 0"
    limit_lower: float = 0.0
    limit_upper: float = 0.0
    limit_effort: float = 0.0
    limit_velocity: float = 0.0
    damping: float = 0.0


@dataclass
class Material:
    name: str = ""
    rgba: str = "0.5 0.5 0.5 1"


@dataclass
class LoopClosure:
    """Represents a closed kinematic chain detected from a virtual link.

    A virtual link is a link that appears as the child of two or more joints.
    In MuJoCo, this is modelled as two sites (one in each parent body frame)
    connected by an equality/connect constraint.
    """
    virtual_link_name: str = ""
    joint1: Optional[Joint] = None      # first joint pointing to the virtual link
    parent1: str = ""                   # parent body of joint1
    joint2: Optional[Joint] = None      # second joint pointing to the virtual link
    parent2: str = ""                   # parent body of joint2
    site1_name: str = ""                # site placed in parent1 at joint1 origin
    site2_name: str = ""                # site placed in parent2 at joint2 origin


# ---------------------------------------------------------------------------
# URDF Parser
# ---------------------------------------------------------------------------

def parse_origin(elem):
    """Extract xyz and rpy from an <origin> element."""
    if elem is None:
        return "0 0 0", "0 0 0"
    xyz = elem.get("xyz", "0 0 0")
    rpy = elem.get("rpy", "0 0 0")
    return xyz, rpy


def parse_geometry(geom_elem):
    """Parse a <geometry> element and return a Geometry dataclass."""
    if geom_elem is None:
        return None
    g = Geometry()
    mesh = geom_elem.find("mesh")
    box = geom_elem.find("box")
    cylinder = geom_elem.find("cylinder")
    sphere = geom_elem.find("sphere")

    if mesh is not None:
        g.geom_type = "mesh"
        g.mesh_filename = mesh.get("filename", "")
        g.mesh_scale = mesh.get("scale", "1 1 1")
    elif box is not None:
        g.geom_type = "box"
        g.box_size = box.get("size", "0.1 0.1 0.1")
    elif cylinder is not None:
        g.geom_type = "cylinder"
        g.cylinder_radius = float(cylinder.get("radius", "0.01"))
        g.cylinder_length = float(cylinder.get("length", "0.1"))
    elif sphere is not None:
        g.geom_type = "sphere"
        g.sphere_radius = float(sphere.get("radius", "0.01"))
    return g


def parse_urdf(urdf_path: str):
    """Parse a URDF file and return links, joints, materials, and robot name."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    robot_name = root.get("name", "robot")

    # --- Materials ---
    materials = {}
    for mat_elem in root.findall("material"):
        name = mat_elem.get("name", "")
        color = mat_elem.find("color")
        rgba = color.get("rgba", "0.5 0.5 0.5 1") if color is not None else "0.5 0.5 0.5 1"
        materials[name] = Material(name=name, rgba=rgba)

    # --- Links ---
    links = {}
    for link_elem in root.findall("link"):
        link = Link(name=link_elem.get("name", ""))

        # Inertial
        inertial_elem = link_elem.find("inertial")
        if inertial_elem is not None:
            inertial = Inertial()
            inertial.origin_xyz, inertial.origin_rpy = parse_origin(inertial_elem.find("origin"))
            mass_elem = inertial_elem.find("mass")
            if mass_elem is not None:
                inertial.mass = float(mass_elem.get("value", "0"))
            inertia_elem = inertial_elem.find("inertia")
            if inertia_elem is not None:
                inertial.ixx = float(inertia_elem.get("ixx", "0"))
                inertial.iyy = float(inertia_elem.get("iyy", "0"))
                inertial.izz = float(inertia_elem.get("izz", "0"))
                inertial.ixy = float(inertia_elem.get("ixy", "0"))
                inertial.iyz = float(inertia_elem.get("iyz", "0"))
                inertial.ixz = float(inertia_elem.get("ixz", "0"))
            link.inertial = inertial

        # Visuals (can be multiple)
        for vis_elem in link_elem.findall("visual"):
            vis = Visual()
            vis.origin_xyz, vis.origin_rpy = parse_origin(vis_elem.find("origin"))
            vis.geometry = parse_geometry(vis_elem.find("geometry"))
            mat = vis_elem.find("material")
            if mat is not None:
                vis.material_name = mat.get("name", "")
                # If inline color, store it
                color = mat.find("color")
                if color is not None and vis.material_name not in materials:
                    rgba = color.get("rgba", "0.5 0.5 0.5 1")
                    materials[vis.material_name] = Material(name=vis.material_name, rgba=rgba)
            link.visuals.append(vis)

        # Collisions (can be multiple)
        for col_elem in link_elem.findall("collision"):
            col = Collision()
            col.origin_xyz, col.origin_rpy = parse_origin(col_elem.find("origin"))
            col.geometry = parse_geometry(col_elem.find("geometry"))
            link.collisions.append(col)

        links[link.name] = link

    # --- Joints ---
    joints = []
    for joint_elem in root.findall("joint"):
        j = Joint()
        j.name = joint_elem.get("name", "")
        j.joint_type = joint_elem.get("type", "fixed")
        j.origin_xyz, j.origin_rpy = parse_origin(joint_elem.find("origin"))

        parent = joint_elem.find("parent")
        child = joint_elem.find("child")
        j.parent = parent.get("link", "") if parent is not None else ""
        j.child = child.get("link", "") if child is not None else ""

        axis = joint_elem.find("axis")
        if axis is not None:
            j.axis = axis.get("xyz", "1 0 0")

        limit = joint_elem.find("limit")
        if limit is not None:
            j.limit_lower = float(limit.get("lower", "0"))
            j.limit_upper = float(limit.get("upper", "0"))
            j.limit_effort = float(limit.get("effort", "0"))
            j.limit_velocity = float(limit.get("velocity", "0"))

        dynamics = joint_elem.find("dynamics")
        if dynamics is not None:
            j.damping = float(dynamics.get("damping", "0"))

        joints.append(j)

    return robot_name, links, joints, materials


# ---------------------------------------------------------------------------
# Tree Builder  (URDF flat → nested tree)
# ---------------------------------------------------------------------------

def _detect_loop_closures(links, joints):
    """
    Detect closed kinematic chains encoded as virtual links.

    A virtual link is any link that appears as the *child* of two or more
    joints.  Each such link produces one LoopClosure record.  The joints
    and the virtual link are removed from the regular tree so that the
    remainder is a valid tree.

    Returns:
        loop_closures:  list[LoopClosure]
        excluded_joints: set of joint names to skip during tree building
        excluded_links:  set of link names to skip
    """
    # Count how many joints claim each link as a child
    child_joint_map = {}   # child_name → [joint, ...]
    for j in joints:
        child_joint_map.setdefault(j.child, []).append(j)

    loop_closures = []
    excluded_joints = set()
    excluded_links = set()

    for child_name, jlist in child_joint_map.items():
        if len(jlist) < 2:
            continue
        # This child is a virtual link — it has ≥ 2 parent joints
        # Take the first two joints as the pair that closes the loop
        j1, j2 = jlist[0], jlist[1]
        lc = LoopClosure(
            virtual_link_name=child_name,
            joint1=j1,
            parent1=j1.parent,
            joint2=j2,
            parent2=j2.parent,
            site1_name=f"{j1.parent}_{child_name}_site1",
            site2_name=f"{j2.parent}_{child_name}_site2",
        )
        loop_closures.append(lc)
        excluded_links.add(child_name)
        for j in jlist:
            excluded_joints.add(j.name)
        print(f"  Loop closure: {child_name}  "
              f"({j1.parent} <-{j1.name}-> {child_name} <-{j2.name}-> {j2.parent})")

    return loop_closures, excluded_joints, excluded_links


def build_kinematic_tree(links, joints):
    """
    Build a kinematic tree from flat URDF links and joints.

    Detects closed kinematic chains (virtual links that are children of 2+
    joints) and separates them into LoopClosure records.  The remaining
    joints form a proper tree.

    Returns:
        root_link_name:  The name of the root link (no parent joint)
        children:        dict mapping parent_name → [(joint, child_link_name), ...]
        joint_for_child: dict mapping child_link_name → joint
        loop_closures:   list[LoopClosure]
        sites_for_body:  dict mapping body_name → [(site_name, pos_xyz), ...]
    """
    loop_closures, excluded_joints, excluded_links = _detect_loop_closures(links, joints)

    # Build the tree from the remaining (non-excluded) joints
    children = {}       # parent_name → [(joint, child_name)]
    child_set = set()   # all links that are children of some tree joint
    joint_for_child = {}

    for j in joints:
        if j.name in excluded_joints:
            continue
        # Skip joints whose parent or child link doesn't exist
        if j.parent not in links or j.child not in links:
            print(f"  Skipping joint '{j.name}': "
                  f"parent '{j.parent}' or child '{j.child}' not in links")
            continue
        if j.child in excluded_links:
            continue

        children.setdefault(j.parent, []).append((j, j.child))
        child_set.add(j.child)
        joint_for_child[j.child] = j

    # Build a mapping: body_name → [(site_name, pos_xyz), ...]
    # so that build_body_xml can emit sites for loop-closure anchors.
    sites_for_body = {}
    for lc in loop_closures:
        sites_for_body.setdefault(lc.parent1, []).append(
            (lc.site1_name, lc.joint1.origin_xyz)
        )
        sites_for_body.setdefault(lc.parent2, []).append(
            (lc.site2_name, lc.joint2.origin_xyz)
        )

    # Root link = any link that is never a child (and not excluded)
    root_candidates = [
        name for name in links
        if name not in child_set and name not in excluded_links
    ]
    if not root_candidates:
        raise ValueError("No root link found — the kinematic tree has a cycle or is empty.")

    # Prefer a root that has children (skip empty placeholder links)
    root_link_name = root_candidates[0]
    for candidate in root_candidates:
        if candidate in children:
            root_link_name = candidate
            break

    return root_link_name, children, joint_for_child, loop_closures, sites_for_body


# ---------------------------------------------------------------------------
# MuJoCo XML Builder
# ---------------------------------------------------------------------------

def fmt(value, decimals=6):
    """Format a float to a fixed number of decimal places, stripping trailing zeros."""
    s = f"{value:.{decimals}f}"
    # Strip trailing zeros but keep at least one decimal
    if '.' in s:
        s = s.rstrip('0').rstrip('.')
    return s


def euler_to_quat(roll, pitch, yaw):
    """Convert RPY (intrinsic XYZ) Euler angles to quaternion (w, x, y, z)."""
    import math
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z


def rpy_to_mujoco_euler(rpy_str):
    """Convert RPY string to MuJoCo euler string. Returns None if all zeros."""
    parts = [float(x) for x in rpy_str.split()]
    if all(abs(v) < 1e-12 for v in parts):
        return None
    return " ".join(fmt(v) for v in parts)


def rpy_to_mujoco_quat(rpy_str):
    """Convert RPY string to MuJoCo quat string. Returns None if identity."""
    parts = [float(x) for x in rpy_str.split()]
    if all(abs(v) < 1e-12 for v in parts):
        return None
    w, x, y, z = euler_to_quat(parts[0], parts[1], parts[2])
    return f"{fmt(w)} {fmt(x)} {fmt(y)} {fmt(z)}"


def collect_meshes(links):
    """Collect all unique mesh filenames and their scales from links."""
    meshes = {}   # mesh_name → (filename, scale)
    for link in links.values():
        for vis in link.visuals:
            if vis.geometry and vis.geometry.geom_type == "mesh":
                fname = vis.geometry.mesh_filename
                name = os.path.splitext(os.path.basename(fname))[0]
                meshes[name] = (fname, vis.geometry.mesh_scale)
        for col in link.collisions:
            if col.geometry and col.geometry.geom_type == "mesh":
                fname = col.geometry.mesh_filename
                name = os.path.splitext(os.path.basename(fname))[0]
                if name not in meshes:
                    meshes[name] = (fname, col.geometry.mesh_scale)
    return meshes


def add_geom_attributes(geom_elem, geometry, origin_xyz, origin_rpy):
    """Add geometry-specific attributes to a MuJoCo <geom> element."""
    pos_str = origin_xyz
    quat_str = rpy_to_mujoco_quat(origin_rpy)

    if geometry.geom_type == "mesh":
        mesh_name = os.path.splitext(os.path.basename(geometry.mesh_filename))[0]
        geom_elem.set("type", "mesh")
        geom_elem.set("mesh", mesh_name)
    elif geometry.geom_type == "box":
        geom_elem.set("type", "box")
        # URDF box size is full extents, MuJoCo is half-extents
        extents = [float(x) for x in geometry.box_size.split()]
        half = [fmt(v / 2) for v in extents]
        geom_elem.set("size", " ".join(half))
    elif geometry.geom_type == "cylinder":
        geom_elem.set("type", "cylinder")
        geom_elem.set("size", f"{fmt(geometry.cylinder_radius)} {fmt(geometry.cylinder_length / 2)}")
    elif geometry.geom_type == "sphere":
        geom_elem.set("type", "sphere")
        geom_elem.set("size", fmt(geometry.sphere_radius))

    if pos_str and not all(abs(float(x)) < 1e-12 for x in pos_str.split()):
        geom_elem.set("pos", pos_str)
    if quat_str:
        geom_elem.set("quat", quat_str)


def build_body_xml(parent_xml, link_name, links, children, joint_for_child,
                   materials, sites_for_body, is_root=False,
                   body_pos="0 0 0", body_quat=None):
    """
    Recursively build nested <body> elements for MuJoCo.

    Args:
        parent_xml:      The parent XML element to attach this body to.
        link_name:       The name of the current URDF link.
        links:           Dict of all links.
        children:        Dict mapping parent → [(joint, child_name), ...]
        joint_for_child: Dict mapping child_name → joint.
        materials:       Dict of materials.
        sites_for_body:  Dict mapping body_name → [(site_name, pos_xyz), ...]
        is_root:         Whether this is the root body (no joint).
        body_pos:        The position of this body in parent frame.
        body_quat:       The orientation quaternion of this body (if non-identity).
    """
    link = links.get(link_name)
    if link is None:
        return

    # Create <body>
    body_elem = ET.SubElement(parent_xml, "body")
    body_elem.set("name", link_name)
    body_elem.set("pos", body_pos)
    if body_quat:
        body_elem.set("quat", body_quat)

    # --- Joint (if not root) ---
    if not is_root and link_name in joint_for_child:
        j = joint_for_child[link_name]
        if j.joint_type != "fixed":
            joint_elem = ET.SubElement(body_elem, "joint")
            joint_elem.set("name", j.name)

            if j.joint_type in ("revolute", "continuous"):
                joint_elem.set("type", "hinge")
            elif j.joint_type == "prismatic":
                joint_elem.set("type", "slide")

            # Joint is at origin of the child body frame → pos="0 0 0"
            joint_elem.set("pos", "0 0 0")
            joint_elem.set("axis", j.axis)

            if j.joint_type == "revolute":
                joint_elem.set("range", f"{fmt(j.limit_lower)} {fmt(j.limit_upper)}")
                joint_elem.set("limited", "true")
            elif j.joint_type == "prismatic":
                joint_elem.set("range", f"{fmt(j.limit_lower)} {fmt(j.limit_upper)}")
                joint_elem.set("limited", "true")

            if j.damping > 0:
                joint_elem.set("damping", fmt(j.damping))

    # --- Inertial ---
    if link.inertial and link.inertial.mass > 0:
        inertial = link.inertial
        inertial_elem = ET.SubElement(body_elem, "inertial")
        inertial_elem.set("pos", inertial.origin_xyz)
        inertial_elem.set("mass", fmt(inertial.mass))
        # MuJoCo fullinertia order: ixx iyy izz ixy ixz iyz
        # URDF order:               ixx iyy izz ixy iyz ixz
        fullinertia = (
            f"{fmt(inertial.ixx)} {fmt(inertial.iyy)} {fmt(inertial.izz)} "
            f"{fmt(inertial.ixy)} {fmt(inertial.ixz)} {fmt(inertial.iyz)}"
        )
        inertial_elem.set("fullinertia", fullinertia)

        quat = rpy_to_mujoco_quat(inertial.origin_rpy)
        if quat:
            inertial_elem.set("quat", quat)

    # --- Visual geoms ---
    for vis in link.visuals:
        if vis.geometry:
            geom_elem = ET.SubElement(body_elem, "geom")
            add_geom_attributes(geom_elem, vis.geometry, vis.origin_xyz, vis.origin_rpy)
            if vis.material_name and vis.material_name in materials:
                geom_elem.set("material", vis.material_name)
            geom_elem.set("contype", "0")
            geom_elem.set("conaffinity", "0")
            geom_elem.set("group", "1")

    # --- Collision geoms ---
    for col in link.collisions:
        if col.geometry:
            geom_elem = ET.SubElement(body_elem, "geom")
            add_geom_attributes(geom_elem, col.geometry, col.origin_xyz, col.origin_rpy)
            geom_elem.set("contype", "2")
            geom_elem.set("conaffinity", "1")

    # --- Sites for loop-closure constraints ---
    if link_name in sites_for_body:
        for site_name, site_pos in sites_for_body[link_name]:
            site_elem = ET.SubElement(body_elem, "site")
            site_elem.set("name", site_name)
            site_elem.set("pos", site_pos)
            site_elem.set("size", "0.01")

    # --- Recurse into children ---
    if link_name in children:
        for child_joint, child_name in children[link_name]:
            child_pos = child_joint.origin_xyz
            child_quat = rpy_to_mujoco_quat(child_joint.origin_rpy)
            build_body_xml(
                body_elem, child_name, links, children, joint_for_child,
                materials, sites_for_body,
                is_root=False, body_pos=child_pos, body_quat=child_quat,
            )


def generate_mjcf(robot_name, links, joints, materials, meshdir="meshes/"):
    """
    Generate a MuJoCo XML ElementTree from parsed URDF data.

    Returns an xml.etree.ElementTree.Element (the <mujoco> root).
    """
    (root_link, children, joint_for_child,
     loop_closures, sites_for_body) = build_kinematic_tree(links, joints)

    # Collect names of joints consumed by loop closures (not real actuators)
    loop_joint_names = set()
    for lc in loop_closures:
        loop_joint_names.add(lc.joint1.name)
        loop_joint_names.add(lc.joint2.name)

    # --- <mujoco> root ---
    mujoco = ET.Element("mujoco")
    mujoco.set("model", robot_name)

    # --- <compiler> ---
    compiler = ET.SubElement(mujoco, "compiler")
    compiler.set("angle", "radian")
    if meshdir:
        compiler.set("meshdir", meshdir)

    # --- <option> ---
    option = ET.SubElement(mujoco, "option")
    option.set("timestep", "0.002")
    option.set("integrator", "RK4")

    # --- <visual> ---
    visual = ET.SubElement(mujoco, "visual")
    headlight = ET.SubElement(visual, "headlight")
    headlight.set("ambient", "0.4 0.4 0.4")
    headlight.set("diffuse", "0.8 0.8 0.8")
    headlight.set("specular", "0.1 0.1 0.1")

    # --- <asset> ---
    asset = ET.SubElement(mujoco, "asset")

    # Skybox
    tex_sky = ET.SubElement(asset, "texture")
    tex_sky.set("type", "skybox")
    tex_sky.set("builtin", "gradient")
    tex_sky.set("rgb1", "0.3 0.5 0.7")
    tex_sky.set("rgb2", "0 0 0")
    tex_sky.set("width", "512")
    tex_sky.set("height", "512")

    # Ground texture and material
    tex_grid = ET.SubElement(asset, "texture")
    tex_grid.set("name", "grid")
    tex_grid.set("type", "2d")
    tex_grid.set("builtin", "checker")
    tex_grid.set("rgb1", "0.1 0.2 0.3")
    tex_grid.set("rgb2", "0.2 0.3 0.4")
    tex_grid.set("width", "300")
    tex_grid.set("height", "300")
    tex_grid.set("mark", "edge")
    tex_grid.set("markrgb", "0.2 0.3 0.4")

    mat_grid = ET.SubElement(asset, "material")
    mat_grid.set("name", "grid")
    mat_grid.set("texture", "grid")
    mat_grid.set("texrepeat", "1 1")
    mat_grid.set("texuniform", "true")
    mat_grid.set("reflectance", "0.2")

    # URDF materials
    for mat in materials.values():
        mat_elem = ET.SubElement(asset, "material")
        mat_elem.set("name", mat.name)
        mat_elem.set("rgba", mat.rgba)

    # Mesh assets
    meshes = collect_meshes(links)
    for mesh_name, (filename, scale) in sorted(meshes.items()):
        mesh_elem = ET.SubElement(asset, "mesh")
        mesh_elem.set("name", mesh_name)
        mesh_elem.set("file", os.path.basename(filename))
        if scale and scale != "1 1 1":
            mesh_elem.set("scale", scale)

    # --- <worldbody> ---
    worldbody = ET.SubElement(mujoco, "worldbody")

    # Spotlight
    light = ET.SubElement(worldbody, "light")
    light.set("name", "spotlight")
    light.set("mode", "targetbody")
    light.set("target", root_link if root_link in links else list(links.keys())[0])
    light.set("diffuse", "0.9 0.9 0.9")
    light.set("specular", "0.3 0.3 0.3")
    light.set("pos", "0 -4 4")
    light.set("dir", "0 1 -1")

    # Build the kinematic tree (sites_for_body handles loop-closure sites)
    build_body_xml(
        worldbody, root_link, links, children, joint_for_child,
        materials, sites_for_body, is_root=True, body_pos="0 0 0",
    )

    # --- <equality> (loop-closure connect constraints) ---
    if loop_closures:
        equality = ET.SubElement(mujoco, "equality")
        for lc in loop_closures:
            connect = ET.SubElement(equality, "connect")
            connect.set("name", f"close_{lc.virtual_link_name}")
            connect.set("site1", lc.site1_name)
            connect.set("site2", lc.site2_name)

    # --- <actuator> (one position actuator per non-fixed, non-loop joint) ---
    actuated_joints = [
        j for j in joints
        if j.joint_type in ("revolute", "continuous", "prismatic")
        and j.name not in loop_joint_names
        and j.parent in links and j.child in links
    ]
    if actuated_joints:
        actuator = ET.SubElement(mujoco, "actuator")
        for j in actuated_joints:
            act = ET.SubElement(actuator, "position")
            act.set("name", f"motor_{j.name}")
            act.set("joint", j.name)
            act.set("ctrlrange", f"{fmt(j.limit_lower)} {fmt(j.limit_upper)}")
            act.set("ctrllimited", "true")

    return mujoco


# ---------------------------------------------------------------------------
# Pretty-print & Write
# ---------------------------------------------------------------------------

def prettify(elem):
    """Return a pretty-printed XML string for the Element."""
    rough = ET.tostring(elem, encoding="unicode")
    parsed = minidom.parseString(rough)
    pretty = parsed.toprettyxml(indent="  ")
    # Remove the extra XML declaration minidom adds (we'll write our own)
    lines = pretty.split("\n")
    # Skip the first line if it's an xml declaration
    if lines and lines[0].startswith("<?xml"):
        lines = lines[1:]
    # Remove blank lines
    lines = [l for l in lines if l.strip()]
    return '<?xml version="1.0" ?>\n' + "\n".join(lines) + "\n"


def convert_urdf_to_mjcf(urdf_path, output_path=None, meshdir="meshes/"):
    """
    Main conversion entry point.

    Args:
        urdf_path:   Path to the input URDF file.
        output_path: Path for the output MJCF XML file (default: same dir, .xml).
        meshdir:     The meshdir path to set in <compiler>.
    """
    if output_path is None:
        base = os.path.splitext(urdf_path)[0]
        output_path = base + ".xml"

    print(f"Parsing URDF: {urdf_path}")
    robot_name, links, joints, materials = parse_urdf(urdf_path)

    print(f"  Robot name : {robot_name}")
    print(f"  Links      : {len(links)}")
    print(f"  Joints     : {len(joints)}")
    print(f"  Materials  : {len(materials)}")

    mujoco_xml = generate_mjcf(robot_name, links, joints, materials, meshdir=meshdir)
    xml_str = prettify(mujoco_xml)

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(xml_str)

    print(f"  Written    : {output_path}")
    return output_path


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Convert a URDF file to MuJoCo MJCF XML."
    )
    parser.add_argument("input", help="Path to the input URDF file")
    parser.add_argument("output", nargs="?", default=None,
                        help="Path for the output MJCF XML (default: <input>.xml)")
    parser.add_argument("--meshdir", default="meshes/",
                        help="meshdir path for the MuJoCo <compiler> element (default: meshes/)")
    args = parser.parse_args()

    convert_urdf_to_mjcf(args.input, args.output, args.meshdir)


if __name__ == "__main__":
    main()
