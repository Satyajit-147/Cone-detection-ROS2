from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare("four_wheel_bot")
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "four_wheel_bot.xacro"])
    world_file = PathJoinSubstitution([pkg_share, "worlds", "cone.world"])  
    rviz_config_file = PathJoinSubstitution([pkg_share, "rviz", "robot_config.rviz"])

    return LaunchDescription([
        # Launch Gazebo with the cone world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"
                ])
            ]),
            launch_arguments={"world": world_file}.items()
        ),

        # Publish joint states (so wheels show up in RViz)
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher",
            output="screen"
        ),

        # Publish robot description and TF
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "robot_description": Command([
                    FindExecutable(name="xacro"), " ", xacro_file
                ]),
                "publish_fixed_joints": True
            }]            
        ),

        # Spawn the robot in Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "four_wheel_bot",
                "-topic", "robot_description",
                "-x", "0", "-y", "0", "-z", "0.1"
            ],
            output="screen"
        ),

        # Node to stop robot based on LIDAR input
        Node(
            package="four_wheel_bot",
            executable="stopper",
            name="obstacle_stopper",
            output="screen"
        ),

        # Orange cone detector node
        Node(
            package="four_wheel_bot",
            executable="orange_detector",
            name="orange_detector",
            output="screen"
        ),

        # RViz2 for visualization
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file]
        )
    ])

