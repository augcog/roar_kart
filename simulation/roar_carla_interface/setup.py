from setuptools import setup
import os
from glob import glob

package_name = "roar_carla_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "param"), glob("param/*.*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="michael",
    maintainer_email="wuxiaohua1011@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "carla_pid_node = roar_carla_interface.carla_pid_node:main",
            "roar_carla_state_publisher = roar_carla_interface.roar_carla_state_publisher:main",
            "carla_depth_img_converter_node = roar_carla_interface.carla_depth_img_converter_node:main",
            "roar_carla_converter_node = roar_carla_interface.roar_to_carla_converter_node:main",
        ],
    },
)
