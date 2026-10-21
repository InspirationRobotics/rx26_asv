from setuptools import find_packages, setup

package_name = "crusader_groundstation"

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
    description="Crusader ground station: gcs_bridge on the boat, ocs_client",
    url="https://github.com/InspirationRobotics/rx26_asv",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gcs_bridge = crusader_groundstation.gcs_bridge:main",
            "ocs_client = crusader_groundstation.ocs_client_node:main",
        ],
    },
)
