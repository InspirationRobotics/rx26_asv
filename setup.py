"""ament_python packaging for the rx26_asv ROS 2 package (Crusader nodes).

NOTE (merge caveat): the live Jetson repo may already carry its own setup.py
(README_PHASE0.md merge procedure uses `rsync --ignore-existing`, so this file
will NOT overwrite it). If the Jetson repo has one, reconcile by copying the
`entry_points` block below into it rather than replacing the file.

Because this file makes the repo ROOT a colcon package, plain `colcon build`
from the root will no longer descend into `interfaces/`. Use the blessed
rebuild path (tools/scripts/rebuild.sh), which builds both explicitly:
    colcon build --symlink-install --base-paths . interfaces
"""
from setuptools import find_packages, setup

package_name = "rx26_asv"

setup(
    name=package_name,
    version="0.4.0",
    packages=find_packages(include=[package_name, package_name + ".*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # single source of truth for ROS-side params (Phase 3.5) — installed to
        # share so launch files can reference it via get_package_share_directory
        ("share/" + package_name + "/config", [
            "config/crusader_params.yaml",
            "config/crusader_devices.json",
            # Livox MID360 network/extrinsic config — repo copy is the source of
            # truth; lidar_fusion.launch.py points the driver at this share path.
            "config/MID360_config.json",
        ]),
        # launch files (referenced via get_package_share_directory)
        ("share/" + package_name + "/launch", [
            "launch/core.launch.py",
            # perception: single-sensor launches (independently runnable) + the
            # composed fused stack that includes both.
            "launch/camera.launch.py",
            "launch/lidar.launch.py",
            "launch/lidar_fusion.launch.py",
        ]),
    ],
    package_data={
        # canonical-label class map consumed by api/perception/detector.py
        package_name + ".api.perception": ["config/*.json"],
    },
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Team Inspiration",
    maintainer_email="brandont3927@gmail.com",
    description="Crusader USV ROS 2 nodes for RobotX 2026",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Phase 1 — HAL / safety
            "telemetry_bridge = rx26_asv.api.navigation.telemetry_bridge:main",
            "frame_transform = rx26_asv.api.navigation.frame_transform:main",
            "rc_override_smoke = rx26_asv.api.testing.rc_override_smoke:main",
            # Phase 2 — perception
            "perception_node = rx26_asv.api.perception.perception_node:main",
            "oakd_guard = rx26_asv.api.perception.oakd_guard:main",
            "lidar_fusion_node = rx26_asv.api.perception.lidar_fusion_node:main",
            # Phase 3 — occupancy + reactive avoidance
            "occupancy_grid_node = rx26_asv.api.navigation.occupancy_grid_node:main",
            "roa_apf_node = rx26_asv.api.navigation.roa_apf_node:main",
            # Phase 4 — mission planner + RoboCommand
            "mission_planner_node = rx26_asv.api.mission.mission_planner_node:main",
            # Operational stack (ported from the Crusader boat repo, rewired to
            # consume telemetry_bridge topics / actuate via the sanctioned paths)
            "led_node = rx26_asv.api.led.led_node:main",
            "pixhawk_led_status_node = rx26_asv.api.pixhawk.pixhawk_led_status_node:main",
            "gate_navigator = rx26_asv.api.navigation.gate_navigator:main",
            "dp_hold = rx26_asv.api.navigation.dp_hold:main",
            "rc_watchdog = rx26_asv.api.safety.rc_heartbeat_watchdog:main",
            # Mission-3 effectors + inter-vehicle comms (ported from RoboBoat,
            # rewired to this repo's conventions)
            "actuator_node = rx26_asv.api.actuators.actuator_node:main",
            "ivc_node = rx26_asv.api.ivc.ivc_node:main",
        ],
    },
)
