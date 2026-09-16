from glob import glob
from setuptools import find_packages, setup


package_name = "drone_gnc"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MA6224 team",
    maintainer_email="maintainer@example.com",
    description="Independent dynamics, EKF, NMPC and logging nodes for the course project.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ekf_node = drone_gnc.ekf_node:main",
            "logger_node = drone_gnc.logger_node:main",
            "nmpc_node = drone_gnc.nmpc_node:main",
            "sensor_simulator_node = drone_gnc.sensor_simulator_node:main",
            "trajectory_node = drone_gnc.trajectory_node:main",
        ]
    },
)

