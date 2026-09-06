from setuptools import find_packages, setup

package_name = "crusader_perception"

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
    description="Crusader perception: OAK-D frames, detection and ranging",
    url="https://github.com/InspirationRobotics/rx26_asv",
    license="MIT",
    entry_points={
        "console_scripts": [
            "oakd_publisher = crusader_perception.oakd_publisher:main",
            "buoy_detector = crusader_perception.buoy_detector:main",
            "lidar_cluster_node = crusader_perception.lidar_cluster_node:main",
            "oak_detector = crusader_perception.oak_detector:main",
            "proximity_bridge = crusader_perception.proximity_bridge_node:main",
        ],
    },
)
