from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def launch_setup(context, *args, **kwargs):
    lidar_param_path = LaunchConfiguration("lidar_param_path").perform(context)
    cfg = yaml.safe_load(open(lidar_param_path, "r"))

    name = cfg["name"]

    composable_nodes = []
    if LaunchConfiguration("use_laserscan_convert").perform(context).lower() == "true":
        for i in range(len(name)):
            print("launching laserscan to pointcloud2 converter for:", name[i])
            composable_nodes.append(
                ComposableNode(
                    package="natto_lidar_converter",
                    plugin="laserscan_to_pointcloud2::laserscan_to_pointcloud2",
                    name=f"{name[i]}_converter",
                    namespace="/sensing/lidar",
                    parameters=[{
                        "frame_id": f"lidar_{name[i]}",
                    }],
                    remappings=[
                        ("laserscan", f"/sensing/lidar/laserscan/{name[i]}"),
                        ("pointcloud2", f"/sensing/lidar/pointcloud2/{name[i]}"),
                    ],
                )
            )
    
    composable_nodes.append(
        ComposableNode(
            package="natto_lidar_merger",
            plugin="pointcloud2_merger::pointcloud2_merger",
            name="pointcloud2_merger",
            namespace="/sensing/lidar",
            parameters=[{
                "frame_id": "base_link",
                "lidar_topics": [f"/sensing/lidar/pointcloud2/{n}" for n in name],
                "publish_frequency": float(LaunchConfiguration("publish_frequency").perform(context)),
            }],
            remappings=[
                ("merged_pointcloud2", "/sensing/lidar/pointcloud2/merged"),
                ("footprint", "/robot/footprint"),
            ],
        )
    )

    print([f"/sensing/lidar/pointcloud2/{n}" for n in name])

    if LaunchConfiguration("use_ransac").perform(context).lower() == "true":
        composable_nodes.append(
            ComposableNode(
                package="natto_ransac",
                plugin="line_detector::line_detector",
                name="line_detector",
                namespace="/sensing/lidar",
                parameters=[{
                    "max_iterations": 30,
                    "distance_threshold": 0.01,
                    "min_inliers": 150,
                    "max_lines": 5,
                }],
                remappings=[
                    ("pointcloud2", "/sensing/lidar/pointcloud2/merged"),
                ],
            )
        )

    container = ComposableNodeContainer(
        name="lidar_converter_container",
        namespace="/sensing/lidar",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        output="screen"
    )

    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("lidar_param_path", default_value=""),
        DeclareLaunchArgument("use_laserscan_convert", default_value="true"),
        DeclareLaunchArgument("use_ransac", default_value="true"),
        DeclareLaunchArgument("publish_frequency", default_value="40.0"),
        OpaqueFunction(function=launch_setup),
    ])
