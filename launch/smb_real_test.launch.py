from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
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
        DeclareLaunchArgument('cloud_topic', default_value='/rslidar/points'),
        DeclareLaunchArgument('odometry_topic', default_value='/graph_msf/est_odometry_odom_imu'),
        DeclareLaunchArgument('pose_stamped_topic', default_value='no_pose_stamped_topic'),
        DeclareLaunchArgument('pose_stamped_with_covariance_topic', default_value='empty'),
        DeclareLaunchArgument('launch_prefix', default_value=''),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('distance_cutoff', default_value='0.2'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value='/opt/ros/jazzy/share/rviz2/rviz/default_config.rviz'),
    ]


    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("smb_bringup"),
                "launch",
                "sensors.launch.py"
            ])
        ]),
    )
    
    # static_tf_map_to_odom = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_map_to_odom",
    #     arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    #     output="log",
    # )

    kinematics_controller = Node(
        package="smb_kinematics",
        executable="smb_kinematics_node",
        name="smb_kinematics_node",
        output="screen",
    )

    low_level_controller = Node(
        package="smb_low_level_controller_gazebo",
        executable="smb_low_level_controller_gazebo_node",
        name="smb_low_level_controller_gazebo_node",
        output="screen",
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

    description_file = PathJoinSubstitution(
        [FindPackageShare("smb_description"), "urdf", "smb.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", description_file]),
        value_type=str
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    open3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("open3d_slam_ros"),
                "launch",
                "open3d_launch.py"
            ])
        ),
        launch_arguments={
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'launch_prefix': LaunchConfiguration('launch_prefix'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'parameter_filename': PathJoinSubstitution([
                FindPackageShare("smb_bringup"),
                "param",
                "smb_challenge_params.lua"
            ])
        }.items(),
    )
    
    # relay_odom_to_dlio = Node(
    #     package="topic_tools",
    #     executable="relay",
    #     name="relay_odom_to_dlio",
    #     arguments=["/odom", "/state_estimation"],
    #     output="screen",
    # )

    # local_odometry = Node(
    #     package="smb_kinematics",
    #     executable="smb_global_to_local_odometry",
    #     name="smb_global_to_local_odometry",
    #     output="screen",
    # )
    
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
    
    return LaunchDescription([
        *launch_args,
        # sensors,
        # kinematics_controller,
        # low_level_controller,
        # terrain_analysis,
        # terrain_analysis_ext,
        robot_state_publisher_node,
        joint_state_publisher_node,
        open3d_launch,
        # relay_odom_to_dlio,
        # local_odometry,
        # static_tf_map_to_odom,
        # far_planner_launch,
        # local_planner_launch,
        # twist_pid,
        # twist_mux,
        rviz2,
    ])