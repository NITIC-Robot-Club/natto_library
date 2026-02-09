from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml

def launch_setup(context, *args, **kwargs):
    lidar_param_path = LaunchConfiguration("lidar_param_path").perform(context)
    cfg = yaml.safe_load(open(lidar_param_path, "r"))

    name = cfg["name"]
    x    = cfg["x"]
    y    = cfg["y"]
    z    = cfg["z"]
    yaw  = cfg["yaw_deg"]
    freq = cfg["frequency"]
    rate = cfg["point_rate"]

    composable_nodes = []
    for i in range(len(name)):
        composable_nodes.append(
            ComposableNode(
                package="natto_simple_simulation",
                plugin="lidar_simulator::lidar_simulator",
                name=name[i],
                namespace="/simulation/lidar",
                parameters=[{
                    "position_x": float(x[i]),
                    "position_y": float(y[i]),
                    "position_z": float(z[i]),
                    "angle_deg": float(yaw[i]),
                    "scan_frequency": float(freq[i]),
                    "frame_id": f"lidar_{name[i]}",
                    "point_rate": int(rate[i]),
                }],
                remappings=[
                    ("simulation_pose", "/simulation/pose"),
                    ("map", "/common/map"),
                    ("laserscan", f"/sensing/lidar/laserscan/{name[i]}"),
                ],
            )
        )

    container = ComposableNodeContainer(
        name="lidar_simulation_container",
        namespace="/simulation",
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
