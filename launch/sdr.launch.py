from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    autostart = LaunchConfiguration("autostart_sdr")

    config_file_cmd = DeclareLaunchArgument(
        name="config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("system_data_recorder"), "config", "sdr.yaml"]
        ),
        description=(
            "Path to the YAML parameter file for the SDR node. "
            "Defaults to the package-installed config/sdr.yaml."
        ),
    )

    autostart_cmd = DeclareLaunchArgument(
        name="autostart_sdr",
        default_value="true",
        choices=["true", "false"],
        description=(
            "When true, the node automatically triggers the CONFIGURE lifecycle "
            "transition ~1 second after startup."
        ),
    )

    sdr_node = Node(
        package="system_data_recorder",
        executable="system_data_recorder",
        name="sdr",
        output="screen",
        parameters=[
            config_file,
            {"autostart": autostart},
        ],
    )

    return LaunchDescription(
        [
            config_file_cmd,
            autostart_cmd,
            sdr_node,
        ]
    )
