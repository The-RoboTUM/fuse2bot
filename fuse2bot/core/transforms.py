

import math


ISAAC_ROTATION_RPY = (math.pi / 2.0, 0.0, 0.0)


def fusion_y_up_to_isaac_z_up_xyz(vector):
    """Rotate a vector from Fusion +Y-up coordinates to Isaac +Z-up coordinates."""
    x, y, z = vector
    return [x, -z, y]


def fusion_y_up_to_isaac_z_up_inertia(inertia):
    """Rotate inertia tensor components from Fusion +Y-up to Isaac +Z-up.

    The project stores inertia as [ixx, iyy, izz, ixy, iyz, ixz].
    """
    ixx, iyy, izz, ixy, iyz, ixz = inertia
    return [
        ixx,
        izz,
        iyy,
        -ixz,
        -iyz,
        ixy,
    ]


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into 
    that about center of mass coordinate


    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]
    
    
    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2+z**2, x**2+z**2, x**2+y**2,
                               -x*y,      -y*z,       -x*z]
    return [ round(i - mass*t, 6) for i, t in zip(inertia, translation_matrix)]
