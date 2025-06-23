from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition, IfCondition
import os


def generate_launch_description():
    default_config_topics = os.path.join(get_package_share_directory('smb_bringup'), 'config', 'twist_mux_topics.yaml')
    launch_args = [
        DeclareLaunchArgument(
            'config_topics',
            default_value=default_config_topics,
            description='Default topics config file'
        ),
    ]
    
    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="log",
    )

    kinematics_controller = Node(
        package="smb_kinematics",
        executable="smb_kinematics_node",
        name="smb_kinematics_node",
        output="screen",
    )

    low_level_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("smb_low_level_controller"),
                "launch",
                "speed_control_node.launch.py"
            ])
        ]),
        launch_arguments={
        }.items(),
    )

    terrain_analysis = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrainAnalysis",
        output="screen",
    )

    terrain_analysis_ext = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrainAnalysisExt",
        output="screen",
    )

    dlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("direct_lidar_inertial_odometry"),
                "launch",
                "dlio.launch.py"
            ])
        ]),
        launch_arguments={
            "rviz": "false",
            "pointcloud_topic": "/rslidar/points",
            "imu_topic": "/imu/data_raw",
            "odom_topic": "/state_estimation",
            "registered_pointcloud_topic": "/registered_scan",
        }.items(),
    )
    
    relay_odom_to_dlio = Node(
        package="topic_tools",
        executable="relay",
        name="relay_odom_to_dlio",
        arguments=["/odom", "/state_estimation"],
        output="screen",
    )

    local_odometry = Node(
        package="smb_kinematics",
        executable="smb_global_to_local_odometry",
        name="smb_global_to_local_odometry",
        output="screen",
    )

    exploration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('tare_planner'), '/explore_robotx.launch']),
        launch_arguments={
            "rviz": "false",
        }.items(),
    )
    
    far_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("far_planner"),
                "launch",
                "far_planner.launch"
            ])
        ]),
    )
    
    local_planner_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("local_planner"),
                "launch",
                "local_planner.launch"
            ])
        ]),
    )
    
    twist_pid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("twist_pid_controller"),
                "launch",
                "twist_pid_controller.launch.py"
            ])
        ),
    )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='far_rviz',
        arguments=[
            '-d',
            PathJoinSubstitution([
                get_package_share_directory('smb_bringup'),
                'rviz',
                'debug.rviz'
            ])
        ],
        respawn=False,
    )

    # twist_mux = Node(
    #     package='twist_mux',
    #     executable='twist_mux',
    #     output='screen',
    #     remappings={('/cmd_vel_out', '/cmd_vel')},
    # )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("smb_bringup"),
                "launch",
                "sensors.launch.py"
            ])
        ),
    )

    return LaunchDescription([
        *launch_args,
        kinematics_controller,
        low_level_controller,
        terrain_analysis,
        terrain_analysis_ext,
        dlio_launch,
        relay_odom_to_dlio,
        local_odometry,
        static_tf_map_to_odom,
        far_planner_launch,
        exploration_launch,
        local_planner_launch,
        twist_pid,
        # twist_mux,
        rviz2,
        sensors_launch,
    ])