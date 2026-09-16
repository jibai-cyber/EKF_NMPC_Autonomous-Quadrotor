import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory("drone_gazebo")
    description_share = get_package_share_directory("drone_description")
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    world = os.path.join(gazebo_share, "worlds", "course_world.sdf")
    bridge_config = os.path.join(gazebo_share, "config", "bridge.yaml")
    gnc_config = os.path.join(
        get_package_share_directory("drone_gnc"), "config", "project.yaml"
    )
    model_path = os.path.join(description_share, "models")
    venv_site_packages = "/opt/drone_venv/lib/python3.12/site-packages"

    return LaunchDescription(
        [
            # ament_python console scripts use /usr/bin/python3; expose the
            # container venv packages (CasADi, NumPy) to that interpreter.
            SetEnvironmentVariable(
                "PYTHONPATH",
                venv_site_packages
                + os.pathsep
                + os.environ.get("PYTHONPATH", ""),
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                model_path + os.pathsep + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            ),
            SetEnvironmentVariable(
                "GZ_SIM_SYSTEM_PLUGIN_PATH",
                os.path.join(get_package_prefix("drone_gazebo"), "lib")
                + os.pathsep
                + os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
                ),
                launch_arguments={"gz_args": f"-r -v 3 {world}"}.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                parameters=[{"config_file": bridge_config}],
                output="screen",
            ),
            Node(
                package="drone_gazebo",
                executable="actuator_bridge",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="drone_gnc",
                executable="sensor_simulator_node",
                parameters=[gnc_config, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="drone_gnc",
                executable="trajectory_node",
                parameters=[gnc_config, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="drone_gnc",
                executable="ekf_node",
                parameters=[gnc_config, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="drone_gnc",
                executable="nmpc_node",
                parameters=[gnc_config, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="drone_gnc",
                executable="logger_node",
                parameters=[gnc_config, {"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
