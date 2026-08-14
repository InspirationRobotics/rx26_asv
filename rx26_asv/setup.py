"""ament_python packaging for the rx26_asv ROS 2 package (Crusader nodes).

This repo is standalone and is the single source of truth for the package.
On the Jetson it is cloned to `~/robotx_ws/src/rx26_asv` — one package source
inside a colcon workspace, not the workspace root.

This file sits in <repo>/rx26_asv/, NOT at the repo root, so that the repo root
is not itself a colcon package. That is deliberate: colcon stops descending as
soon as it finds a package, so a package.xml at the repo root would hide the
sibling `interfaces/` package and silently build against stale messages. Keep
it this way — plain `colcon build` from the workspace root finds both.
"""
from setuptools import find_packages, setup

package_name = "rx26_asv"

setup(
    name=package_name,
    version="0.5.0",
    packages=find_packages(include=[package_name, package_name + ".*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # single source of truth for ROS-side params — installed to
        # share so launch files can reference it via get_package_share_directory
        ("share/" + package_name + "/config", ["config/crusader_params.yaml"]),
        # launch files (referenced via get_package_share_directory)
        ("share/" + package_name + "/launch", ["launch/core.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Team Inspiration",
    maintainer_email="brandont3927@gmail.com",
    description="Crusader USV ROS 2 nodes for RobotX 2026",
    url="https://github.com/InspirationRobotics/rx26_asv",
    license="MIT",
    entry_points={
        "console_scripts": [
            # The whole shipped stack — everything here has run on the boat.
            # Nodes are added back only once they have, not once they compile.
            "telemetry_bridge = rx26_asv.api.navigation.telemetry_bridge:main",
            "rc_watchdog = rx26_asv.api.safety.rc_heartbeat_watchdog:main",
            "led_node = rx26_asv.api.led.led_node:main",
            "pixhawk_led_status_node = rx26_asv.api.pixhawk.pixhawk_led_status_node:main",
        ],
    },
)
