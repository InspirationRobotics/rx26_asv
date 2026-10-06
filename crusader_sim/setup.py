import os
from glob import glob

from setuptools import setup

PKG = "crusader_sim"


def data(sub, pattern="*"):
    # files only: a sub-directory in the glob (config/tuning_profiles) fails the install
    return (os.path.join("share", PKG, sub),
            [f for f in glob(os.path.join(sub, pattern)) if os.path.isfile(f)])


setup(
    name=PKG,
    version="0.1.0",
    packages=[PKG],
    # task1_panel serves its page from beside its own .py
    package_data={PKG: ["*.html"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PKG]),
        ("share/" + PKG, ["package.xml"]),
        data("config"),
        data("config/tuning_profiles"),
        data("courses"),
        data("scripts"),
        data("docker"),
        data("setup"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team Inspiration",
    description="Gazebo + SITL simulation of Crusader (sim only)",
    license="MIT",
    entry_points={
        "console_scripts": [
            # ROS nodes (crsd-sim container)
            "livox_shim = crusader_sim.livox_shim:main",
            "sim_camera = crusader_sim.sim_camera:main",
            "panel_feed = crusader_sim.panel_feed:main",
            "task3_world = crusader_sim.task3_world:main",   # Task 3: RoboCommand, eye, water, referee
            "task3_goal = crusader_sim.task3_goal:main",
            # plain processes
            "sim_uav = crusader_sim.sim_uav:main",
            "task1_goal = crusader_sim.task1_goal:main",
            "manual_drive = crusader_sim.manual_drive:main",
            "task1_judge = crusader_sim.task1_judge:main",
            "sim_transmitter = crusader_sim.sim_transmitter:main",
            "check_motion = crusader_sim.check_motion:main",
            "gen_crusader = crusader_sim.gen_crusader:main",
            "gen_world = crusader_sim.gen_world:main",
            "sitl_params = crusader_sim.sitl_params:main",
        ],
    },
)
