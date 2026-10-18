from setuptools import find_packages, setup

package_name = "crusader_world_model"

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
    description="Crusader world model (fusion and occupancy grid)",
    url="https://github.com/InspirationRobotics/rx26_asv",
    license="MIT",
    entry_points={
        "console_scripts": [
            # HAS NOT RUN ON THE BOAT. Listed here, against the letter of README
            # "Format" rule 1 (as lidar_cluster_node is), because a node with no
            # entry point cannot be started by `ros2 run` at all, which makes the
            # bench session that would earn it a place impossible. Not in
            # core.launch.py, and it carries a NOTE at the top of its file.
            "target_tracker = crusader_world_model.target_tracker_node:main",
        ],
    },
)
