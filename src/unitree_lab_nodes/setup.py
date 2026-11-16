from glob import glob
from setuptools import setup

package_name = "unitree_lab_nodes"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "robot_bridge = unitree_lab_nodes.robot_bridge:main",
            "state_aggregator = unitree_lab_nodes.state_aggregator:main",
            "cloud_to_scan = unitree_lab_nodes.cloud_to_scan:main",
            "posture_server = unitree_lab_nodes.posture_server:main",
            "patrol_controller = unitree_lab_nodes.patrol_controller:main",
        ],
    },
)
