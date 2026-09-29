"""gen_crusader — write Crusader's Gazebo model from two YAML files.

    python3 -m crusader_sim.gen_crusader --out <models_dir> \
        [--hull config/crusader_hull.yaml] [--params <crusader_params.yaml>]

Two sources, on purpose:
  crusader_hull.yaml    the hull, mass, damping, thrusters — PLACEHOLDERS until CAD
  crusader_params.yaml  the boat's own sensor extrinsics (lidar_*, cam_*), read
                        from the SAME file the boat's nodes read, so the sim cannot
                        mount a sensor somewhere the perception code does not
                        believe it is.

The ArduPilotPlugin block is what makes this an ArduRover vehicle: SITL runs the
boat's real mixer (FRAME_TYPE=2 OmniX) and sends SERVO1..4 PWM here over JSON;
each PWM becomes a thrust in newtons on one gz Thruster.
"""
import argparse
import math
import os

import yaml

from crusader_sim import sdf_util as S
from crusader_sim.paths import default_hull_yaml, default_params_yaml

RHO_WATER = 1000.0


def _sensor_mounts(params_path):
    with open(params_path, encoding="utf-8") as f:
        p = yaml.safe_load(f)
    lid = p["lidar_cluster_node"]["ros__parameters"]
    cam = p["target_tracker"]["ros__parameters"]
    return {
        "lidar": (float(lid["lidar_x"]), float(lid["lidar_y"]), float(lid["lidar_z"])),
        "lidar_signs": (float(lid["lidar_sign_y"]), float(lid["lidar_sign_z"])),
        "cam": (float(cam["cam_x"]), float(cam["cam_y"]), float(cam["cam_z"])),
        "cam_yaw_deg": float(cam.get("cam_yaw_deg", 0.0)),
        "cam_pitch_deg": float(cam.get("cam_pitch_deg", 0.0)),
    }


T200_MASS_KG = 0.344        # SPEC, in water-less air; one thruster link each


def _hull_links(h, n_thrusters):
    """Visuals + collisions + inertial of base_link.

    mass_kg is the WHOLE boat; the thruster links carry their own share, so
    base_link gets the rest — and the buoyant width is sized on the whole, or
    the boat floats a centimetre deep (it did, on the first run)."""
    L, B, D = h["length_m"], h["beam_m"], h["depth_m"]
    draft, total = h["draft_m"], h["mass_kg"]
    mass = total - n_thrusters * T200_MASS_KG
    yc = h["pontoon_centre_y_m"]
    # Underwater pontoon width that floats the whole boat at exactly `draft`.
    w_buoy = total / (RHO_WATER * 2.0 * L * draft)
    if w_buoy > B / 2.0:
        raise ValueError(f"mass {mass} kg cannot float at draft {draft} m inside a "
                         f"{L}x{B} m hull (needs pontoons {w_buoy:.3f} m wide)")
    out = []
    # Each pontoon is SEGMENTED along its length. gz's graded buoyancy gets a
    # box's submerged VOLUME right but not how its centre of buoyancy slides
    # along a tilted box, so one long box per pontoon has no pitch restoring
    # moment: roll was stable (two pontoons, two volumes) and pitch capsized in
    # ~8 s with the CG at 0.22 m (2026-09-29). Segments restore pitch the same
    # way the two pontoons restore roll — the VRX Surface plugin's trick.
    nseg = int(h.get("buoyancy_segments", 8))
    seg = L / nseg
    for side, y in (("port", yc), ("stbd", -yc)):
        for i in range(nseg):
            x = -L / 2.0 + seg * (i + 0.5)
            out.append(S.collision(f"pontoon_{side}_{i}", S.box(seg, w_buoy, D),
                                   S.pose(x, y, D / 2.0)))
    # thin deck for contact with docks; above the water, so ~no buoyancy
    out.append(S.collision("deck", S.box(L, B, 0.03), S.pose(0, 0, D - 0.015)))

    mesh = h.get("visual_mesh") or ""
    if mesh:
        out.append(f'<visual name="hull_mesh"><geometry><mesh><uri>{mesh}</uri>'
                   f"</mesh></geometry></visual>")
    else:
        wv = h["pontoon_visual_width_m"]
        for side, y in (("port", yc), ("stbd", -yc)):
            out.append(S.visual(f"pontoon_{side}_v", S.box(L, wv, D), "hull",
                                S.pose(0, y, D / 2.0)))
        out.append(S.visual("deck_v", S.box(L * 0.9, B, 0.03), "metal",
                            S.pose(0, 0, D - 0.015)))
        # bow marker so the heading is obvious in the GUI
        out.append(S.visual("bow_v", S.box(0.06, 0.06, 0.02), "orange",
                            S.pose(L / 2.0 - 0.05, 0, D + 0.01)))
    inertial = (f"<inertial>{S.pose(0, 0, h['cg_z_m'])}"
                f"{S.box_inertia(mass, L, B, D)}</inertial>")
    return inertial, "".join(out), w_buoy


def _hydrodynamics(d):
    # Fossen: forces are  X = xU*u + xUabsU*u*|u| ... ; damping coefficients are
    # NEGATIVE. Added mass left at 0: it destabilises small, light hulls in gz
    # and is second-order for waypoint/station-keeping work.
    c = {
        "xU": -d["x_u"], "xUabsU": -d["x_uu"],
        "yV": -d["y_v"], "yVabsV": -d["y_vv"],
        "zW": -d["z_w"], "zWabsW": -d["z_ww"],
        "kP": -d["k_p"], "kPabsP": -d["k_pp"],
        "mQ": -d["m_q"], "mQabsQ": -d["m_qq"],
        "nR": -d["n_r"], "nRabsR": -d["n_rr"],
    }
    body = "".join(f"<{k}>{v:.3f}</{k}>" for k, v in c.items())
    zeros = "".join(f"<{k}>0</{k}>" for k in
                    ("xDotU", "yDotV", "zDotW", "kDotP", "mDotQ", "nDotR"))
    return ('<plugin filename="gz-sim-hydrodynamics-system" '
            'name="gz::sim::systems::Hydrodynamics">'
            f"<link_name>base_link</link_name>{zeros}{body}</plugin>")


def thrust_topic(unit_name):
    return f"/model/crusader/joint/thruster_{unit_name}_joint/cmd_thrust"


def _thrusters(t):
    """Links + joints + one Thruster system per T200, and the matching
    ArduPilotPlugin <control> blocks."""
    links, plugins, controls = [], [], []
    r, ln, m = 0.05, 0.11, T200_MASS_KG             # T200 body, SPEC
    tmax = float(t["max_thrust_n"])
    for u in t["units"]:
        n = u["name"]
        yaw = math.radians(u["yaw_deg"])
        # link x-axis = thrust axis; cylinder visual turned to lie along it
        links.append(
            f'<link name="thruster_{n}">'
            f'{S.pose(u["x_m"], u["y_m"], t["z_m"], 0, 0, yaw)}'
            f"<inertial>{S.cyl_inertia(m, r, ln)}</inertial>"
            f'{S.visual("body", S.cylinder(r, ln), "black", S.pose(0, 0, 0, 0, math.pi / 2, 0))}'
            f'{S.visual("dir", S.box(0.08, 0.01, 0.01), "orange", S.pose(0.06, 0, 0))}'
            "</link>"
            f'<joint name="thruster_{n}_joint" type="revolute">'
            f"<parent>base_link</parent><child>thruster_{n}</child>"
            "<axis><xyz>1 0 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper>"
            "</limit><dynamics><damping>0.0</damping></dynamics></axis></joint>")
        plugins.append(
            '<plugin filename="gz-sim-thruster-system" name="gz::sim::systems::Thruster">'
            f"<namespace>crusader</namespace><joint_name>thruster_{n}_joint</joint_name>"
            "<use_angvel_cmd>false</use_angvel_cmd>"
            "<thrust_coefficient>0.004422</thrust_coefficient>"
            f"<fluid_density>{RHO_WATER}</fluid_density>"
            f"<propeller_diameter>{t['prop_diameter_m']}</propeller_diameter>"
            "<velocity_control>true</velocity_control>"
            f"<max_thrust_cmd>{tmax}</max_thrust_cmd>"
            f"<min_thrust_cmd>{-tmax}</min_thrust_cmd>"
            "</plugin>")
        # PWM [pwm_min, pwm_max] -> raw [0, 1] -> thrust = mult * (raw - 0.5).
        # mult carries pwm_sense: see crusader_hull.yaml.
        mult = 2.0 * tmax * float(u["pwm_sense"])
        controls.append(
            f'<control channel="{int(u["servo"]) - 1}">'
            f"<jointName>thruster_{n}_joint</jointName>"
            "<type>COMMAND</type>"
            f"<cmd_topic>{thrust_topic(n)}</cmd_topic>"
            f"<multiplier>{mult:.3f}</multiplier><offset>-0.5</offset>"
            f"<servo_min>{t['pwm_min']}</servo_min><servo_max>{t['pwm_max']}</servo_max>"
            "</control>")
    return "".join(links), "".join(plugins), "".join(controls)


def _sensors(s, mounts):
    lx, ly, lz = mounts["lidar"]
    sy, sz = mounts["lidar_signs"]
    # sign_y = sign_z = -1  <=>  a 180 deg roll between raw and body frames.
    if (sy, sz) == (-1.0, -1.0):
        lroll = math.pi
    elif (sy, sz) == (1.0, 1.0):
        lroll = 0.0
    else:
        raise ValueError(f"lidar signs {sy},{sz} are not a proper rotation")
    mid = s["mid360"]
    cx, cy, cz = mounts["cam"]
    cyaw = math.radians(mounts["cam_yaw_deg"])        # + = aimed to PORT
    cpitch = math.radians(mounts["cam_pitch_deg"])    # + = aimed DOWN (REP-103 +pitch)
    oak = s["oak_d_lr"]
    hfov = math.radians(oak["hfov_deg"])
    cam_pose = S.pose(cx, cy, cz, 0, cpitch, cyaw)

    lidar = (
        '<sensor name="mid360" type="gpu_lidar">'
        f"{S.pose(lx, ly, lz, lroll, 0, 0)}"
        "<topic>/crusader/mid360</topic><gz_frame_id>livox_frame</gz_frame_id>"
        f"<update_rate>{mid['rate_hz']}</update_rate><always_on>1</always_on>"
        "<lidar><scan>"
        f"<horizontal><samples>{mid['h_samples']}</samples><resolution>1</resolution>"
        f"<min_angle>{-math.pi:.5f}</min_angle><max_angle>{math.pi:.5f}</max_angle></horizontal>"
        f"<vertical><samples>{mid['v_samples']}</samples><resolution>1</resolution>"
        f"<min_angle>{math.radians(mid['v_min_deg']):.5f}</min_angle>"
        f"<max_angle>{math.radians(mid['v_max_deg']):.5f}</max_angle></vertical>"
        "</scan>"
        f"<range><min>{mid['range_min_m']}</min><max>{mid['range_max_m']}</max>"
        "<resolution>0.01</resolution></range>"
        f"<noise><type>gaussian</type><mean>0</mean><stddev>{mid['noise_sd_m']}</stddev></noise>"
        "<visibility_mask>4294967293</visibility_mask>"   # ~0x2: ignore the water plane
        "</lidar></sensor>")
    rgb = (
        '<sensor name="oak_rgb" type="camera">'
        # gz puts camera_info beside the image topic, so each camera needs its
        # own parent or the two resolutions fight over one camera_info topic
        f"{cam_pose}<topic>/crusader/oak/rgb/image</topic>"
        "<gz_frame_id>oak_rgb_camera_optical_frame</gz_frame_id>"
        f"<update_rate>{oak['rate_hz']}</update_rate><always_on>1</always_on>"
        f"<camera><horizontal_fov>{hfov:.5f}</horizontal_fov>"
        f"<image><width>{oak['rgb_width']}</width><height>{oak['rgb_height']}</height>"
        "<format>R8G8B8</format></image>"
        "<clip><near>0.05</near><far>300</far></clip>"
        "<noise><type>gaussian</type><mean>0</mean><stddev>0.004</stddev></noise>"
        "</camera></sensor>")
    depth = (
        '<sensor name="oak_depth" type="depth_camera">'
        f"{cam_pose}<topic>/crusader/oak/depth/image</topic>"
        "<gz_frame_id>oak_rgb_camera_optical_frame</gz_frame_id>"
        f"<update_rate>{oak['rate_hz']}</update_rate><always_on>1</always_on>"
        f"<camera><horizontal_fov>{hfov:.5f}</horizontal_fov>"
        f"<image><width>{oak['depth_width']}</width><height>{oak['depth_height']}</height>"
        "<format>R_FLOAT32</format></image>"
        f"<clip><near>{oak['depth_min_m']}</near><far>{oak['depth_max_m']}</far></clip>"
        "</camera></sensor>")
    imu = (
        '<sensor name="imu_sensor" type="imu">'
        '<pose degrees="true">0 0 0 180 0 0</pose>'
        f"<always_on>1</always_on><update_rate>{s['imu']['rate_hz']}</update_rate>"
        "</sensor>")
    visuals = (
        S.visual("mid360_v", S.cylinder(0.0325, 0.06), "black", S.pose(lx, ly, lz))
        + S.visual("oak_v", S.box(0.05, 0.16, 0.045), "metal", S.pose(cx, cy, cz, 0, cpitch, cyaw))
        + S.visual("mast_v", S.cylinder(0.015, cz - 0.45), "metal",
                   S.pose(cx - 0.04, cy, 0.45 + (cz - 0.45) / 2.0))
    )
    return lidar + rgb + depth + imu, visuals


def _ardupilot_plugin(controls):
    return (
        '<plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">'
        "<fdm_addr>127.0.0.1</fdm_addr><fdm_port_in>9002</fdm_port_in>"
        "<connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>"
        "<lock_step>1</lock_step><have_32_channels>0</have_32_channels>"
        '<modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0'
        "</modelXYZToAirplaneXForwardZDown>"
        '<gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>'
        "<imuName>imu_sensor</imuName>"
        f"{controls}</plugin>")


def build(hull_path, params_path):
    with open(hull_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    mounts = _sensor_mounts(params_path)
    inertial, hull_geo, w_buoy = _hull_links(cfg["hull"], len(cfg["thrusters"]["units"]))
    th_links, th_plugins, controls = _thrusters(cfg["thrusters"])
    sensors, sensor_vis = _sensors(cfg["sensors"], mounts)
    sdf = (
        '<?xml version="1.0"?>\n<sdf version="1.9"><model name="crusader">'
        '<link name="base_link">'
        f"{inertial}{hull_geo}{sensor_vis}{sensors}</link>"
        f"{th_links}"
        f"{_hydrodynamics(cfg['damping'])}{th_plugins}{_ardupilot_plugin(controls)}"
        "</model></sdf>\n")
    return sdf, {"buoyant_pontoon_width_m": round(w_buoy, 4), **mounts}


def write_model(out_dir, hull_path=None, params_path=None):
    hull_path = hull_path or default_hull_yaml()
    params_path = params_path or default_params_yaml()
    sdf, info = build(hull_path, params_path)
    mdir = os.path.join(out_dir, "crusader")
    os.makedirs(mdir, exist_ok=True)
    with open(os.path.join(mdir, "model.sdf"), "w", encoding="utf-8") as f:
        f.write(sdf)
    with open(os.path.join(mdir, "model.config"), "w", encoding="utf-8") as f:
        f.write(S.model_config("crusader", "Crusader USV, generated by gen_crusader.py "
                                            "from crusader_hull.yaml + crusader_params.yaml"))
    return mdir, info


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True, help="models directory to write into")
    ap.add_argument("--hull", default=None)
    ap.add_argument("--params", default=None)
    a = ap.parse_args()
    mdir, info = write_model(a.out, a.hull, a.params)
    print(f"wrote {mdir}")
    for k, v in info.items():
        print(f"  {k}: {v}")


if __name__ == "__main__":
    main()
