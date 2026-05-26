from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    use_composition = LaunchConfiguration('use_composition')

    parameters = [{
        'laser_scan_topic' : '/scan',
        'odom_topic' : '/odom_rf2o',
        'publish_tf' : True,
        'base_frame_id' : 'base_link',
        'odom_frame_id' : 'odom',
        'init_pose_from_topic' : '',
        'freq' : 20.0}]

    return LaunchDescription([
            DeclareLaunchArgument(
                'use_composition',
                default_value='false',
                description='Launch RF2O as a composable node in a component container.'),

            Node(
                condition=UnlessCondition(use_composition),
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=parameters,
            ),

            ComposableNodeContainer(
                condition=IfCondition(use_composition),
                name='rf2o_laser_odometry_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                output='screen',
                composable_node_descriptions=[
                    ComposableNode(
                        package='rf2o_laser_odometry',
                        plugin='rf2o::CLaserOdometry2DNode',
                        name='rf2o_laser_odometry',
                        parameters=parameters,
                    ),
                ],
            ),
    ])
