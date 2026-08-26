from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VALID_VARIANTS = ("v1", "v2", "v3", "v4", "v5", "v6")


def launch_robot(context, package_share):
    variant = LaunchConfiguration("variant").perform(context).lower()
    model_override = LaunchConfiguration("model").perform(context)

    if variant not in VALID_VARIANTS:
        raise ValueError(
            f"Unsupported robot variant '{variant}'. Choose one of: "
            + ", ".join(VALID_VARIANTS)
        )

    model = Path(model_override or package_share / "urdf" / f"standard_{variant}.urdf")
    robot = ET.parse(model).getroot()
    package_prefix = "package://standard/"
    for mesh in robot.findall(".//mesh"):
        uri = mesh.attrib.get("filename", "")
        if uri.startswith(package_prefix):
            resource = package_share / uri.removeprefix(package_prefix)
            mesh.attrib["filename"] = resource.resolve().as_uri()

    # spawn_entity parses a Unicode string, so omit the XML encoding declaration.
    robot_description = ET.tostring(robot, encoding="unicode")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"use_sim_time": True},
            ],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_standard",
            output="screen",
            arguments=[
                "-entity",
                LaunchConfiguration("entity_name"),
                "-topic",
                "robot_description",
                "-x",
                LaunchConfiguration("x"),
                "-y",
                LaunchConfiguration("y"),
                "-z",
                LaunchConfiguration("z"),
                "-Y",
                LaunchConfiguration("yaw"),
            ],
        ),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("standard"))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))

    return LaunchDescription(
        [
            # Gazebo Classic otherwise blocks its GUI while refreshing the
            # online model database when the service is unavailable.
            SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
            DeclareLaunchArgument(
                "variant",
                default_value="v1",
                choices=list(VALID_VARIANTS),
                description=(
                    "Robot form: v1 (full), v2 (dual arm), v3 (patrol), "
                    "v4 (engineering), v5 (single arm), v6 (chassis)"
                ),
            ),
            DeclareLaunchArgument(
                "model",
                default_value="",
                description="Optional absolute URDF path overriding the selected variant",
            ),
            DeclareLaunchArgument(
                "entity_name",
                default_value="standard",
                description="Gazebo entity name",
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Start the Gazebo client window",
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument(
                "z",
                default_value="0.05",
                description="Initial height above the ground plane",
            ),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(gazebo_share / "launch" / "gazebo.launch.py")
                ),
                launch_arguments={
                    "gui": LaunchConfiguration("gazebo_gui")
                }.items(),
            ),
            OpaqueFunction(function=launch_robot, args=[package_share]),
        ]
    )
