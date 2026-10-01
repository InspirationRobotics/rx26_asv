from glob import glob

from setuptools import find_packages, setup

package_name = "crusader_nav"

setup(
    name=package_name,
    version="0.5.0",
    packages=find_packages(include=[package_name, package_name + ".*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Team Inspiration",
    maintainer_email="brandont3927@gmail.com",
    description="Crusader navigation: the shared map frame, and the Nav2 planner stack launch",
    url="https://github.com/InspirationRobotics/rx26_asv",
    license="MIT",
    entry_points={
        "console_scripts": [
            "nav_frames_node = crusader_nav.nav_frames_node:main",
            "costmap_probe = crusader_nav.costmap_probe:main",
        ],
    },
)
