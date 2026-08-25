from setuptools import find_packages, setup

package_name = "crusader_behavior"

setup(
    name=package_name,
    version="0.5.0",
    packages=find_packages(include=[package_name, package_name + ".*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Team Inspiration",
    maintainer_email="brandont3927@gmail.com",
    description="Crusader safety watchdog and LED status stack",
    url="https://github.com/InspirationRobotics/rx26_asv",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rc_watchdog = crusader_behavior.safety.rc_heartbeat_watchdog:main",
            # NOT PROVEN ON THE BOAT, and not in core.launch.py. It is an
            # entry point because the state nobody can start is no state.
            "mission_planner = crusader_behavior.mission.mission_planner:main",
            "led_node = crusader_behavior.indicator.led_node:main",
            "pixhawk_led_status_node = crusader_behavior.indicator.pixhawk_led_status_node:main",
        ],
    },
)
