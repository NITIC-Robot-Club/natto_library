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

    pkg_share = get_package_share_directory("natto_launch")
    yaml_path = os.path.join(pkg_share, "config", lidar_param_path)
    cfg = yaml.safe_load(open(yaml_path, "r"))

    name = cfg["name"]
    x    = cfg["x"]
    y    = cfg["y"]
    yaw  = cfg["yaw_deg"]
    freq = cfg["frequency"]
    rate = cfg["point_rate"]

    composable_nodes = []
    for i in range(len(name)):
        composable_nodes.append(
            ComposableNode(
                package="natto_simple_simulation",
                plugin="lidar_simulator::lidar_simulator",
                name=f"lidar_simulation_{name[i]}",
                namespace="/simulation",
                parameters=[{
                    "position_x": float(x[i]),
                    "position_y": float(y[i]),
                    "angle_deg": float(yaw[i]),
                    "scan_frequency": float(freq[i]),
                    "frame_id": f"lidar_{name[i]}",
                    "point_rate": int(rate[i]),
                }],
                remappings=[
                    ("simulation_pose", "/simulation/pose"),
                    ("map", "/common/map"),
                    ("laser_scan", f"/sensing/lidar/laser_scan/{name[i]}"),
                ],
            )
        )

    container = ComposableNodeContainer(
        name="lidar_container",
        namespace="simulation",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        output="screen"
    )

    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("lidar_param_path", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
