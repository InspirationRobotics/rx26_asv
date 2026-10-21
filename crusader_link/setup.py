from setuptools import setup

package_name = "crusader_link"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Team Inspiration",
    maintainer_email="brandont3927@gmail.com",
    description="Crusader's end of the RFD900 mesh to the UAV",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rfd_link_node = crusader_link.rfd_link_node:main",
        ],
    },
)
