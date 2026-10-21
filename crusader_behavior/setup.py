from setuptools import setup

package_name = "crusader_behavior"

setup(
    name=package_name,
    version="0.6.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Team Inspiration",
    maintainer_email="brandont3927@gmail.com",
    description="Crusader's autopilot gateway, RC-loss watchdog and LED status",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "telemetry_bridge = crusader_behavior.telemetry_bridge:main",
            "rc_watchdog = crusader_behavior.rc_watchdog:main",
            "led_node = crusader_behavior.led_node:main",
            "led_status_node = crusader_behavior.led_status_node:main",
        ],
    },
)
