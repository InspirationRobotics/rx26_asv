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
    description="Crusader ground station: one web page for the whole boat",
    url="https://github.com/InspirationRobotics/rx26_asv",
    license="MIT",
    entry_points={
        "console_scripts": [
            # NOT PROVEN ON THE BOAT. Carries a NOTE header, and is not in
            # core.launch.py. It is an entry point because a ground station
            # that cannot be started is not a ground station.
            "ground_station = crusader_groundstation.gcs_node:main",
        ],
    },
)
