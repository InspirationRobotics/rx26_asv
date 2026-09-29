"""sdf_util — the few SDF fragments every generator here needs.

Plain string building, deliberately: the models are primitives, and a template
engine or xacro would be one more thing to install in the container for no gain.
"""
import math

# Named colours: (r, g, b). Task colours are the handbook's hex values where it
# gives them (3.3.3:13,19 — circles R #E8282B, G #089404, B #0072CE).
COLOURS = {
    "white": (0.95, 0.95, 0.95),
    "grey": (0.80, 0.81, 0.82),          # #CCCED0, platform background
    "black": (0.03, 0.03, 0.03),
    "red": (0.91, 0.16, 0.17),           # #E8282B
    "green": (0.03, 0.58, 0.02),         # #089404
    "blue": (0.00, 0.45, 0.81),          # #0072CE
    "orange": (1.00, 0.45, 0.05),
    "yellow": (0.95, 0.85, 0.10),
    "hull": (0.20, 0.22, 0.25),
    "metal": (0.55, 0.57, 0.60),
    "dock": (0.85, 0.85, 0.80),
    "water": (0.05, 0.25, 0.35),
}


def pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return (f"<pose>{x:.4f} {y:.4f} {z:.4f} "
            f"{roll:.5f} {pitch:.5f} {yaw:.5f}</pose>")


def material(colour, emissive=0.0, alpha=1.0):
    r, g, b = COLOURS[colour] if isinstance(colour, str) else colour
    e = (r * emissive, g * emissive, b * emissive)
    tr = "" if alpha >= 1.0 else f"<transparency>{1.0 - alpha:.2f}</transparency>"
    return (f"<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>"
            f"<diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular>"
            f"<emissive>{e[0]:.3f} {e[1]:.3f} {e[2]:.3f} 1</emissive></material>{tr}")


def box(sx, sy, sz):
    return f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>"


def cylinder(radius, length):
    return (f"<geometry><cylinder><radius>{radius:.4f}</radius>"
            f"<length>{length:.4f}</length></cylinder></geometry>")


def sphere(radius):
    return f"<geometry><sphere><radius>{radius:.4f}</radius></sphere></geometry>"


def visual(name, geometry, colour, p="", emissive=0.0, alpha=1.0, flags=None):
    vf = "" if flags is None else f"<visibility_flags>{flags}</visibility_flags>"
    return (f'<visual name="{name}">{p}{geometry}'
            f"{material(colour, emissive, alpha)}{vf}</visual>")


def collision(name, geometry, p=""):
    """Suffixed, so a builder can give a part's visual and collision the same
    name: SDF wants every name in a link unique across BOTH (gz sim loads it
    anyway, `gz sdf -k` rejects it)."""
    return f'<collision name="{name}_col">{p}{geometry}</collision>'


def box_inertia(m, sx, sy, sz):
    """Inertia block of a solid box, about its own centre."""
    ixx = m * (sy * sy + sz * sz) / 12.0
    iyy = m * (sx * sx + sz * sz) / 12.0
    izz = m * (sx * sx + sy * sy) / 12.0
    return (f"<mass>{m:.4f}</mass><inertia><ixx>{ixx:.5f}</ixx><ixy>0</ixy>"
            f"<ixz>0</ixz><iyy>{iyy:.5f}</iyy><iyz>0</iyz>"
            f"<izz>{izz:.5f}</izz></inertia>")


def cyl_inertia(m, r, length):
    """Solid cylinder, axis along its local z."""
    ixx = m * (3 * r * r + length * length) / 12.0
    izz = m * r * r / 2.0
    return (f"<mass>{m:.4f}</mass><inertia><ixx>{ixx:.6f}</ixx><ixy>0</ixy>"
            f"<ixz>0</ixz><iyy>{ixx:.6f}</iyy><iyz>0</iyz>"
            f"<izz>{izz:.6f}</izz></inertia>")


def model_config(name, description):
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Team Inspiration (generated)</name></author>
  <description>{description}</description>
</model>
"""


def rad(deg):
    return math.radians(deg)
