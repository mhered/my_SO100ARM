from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable

from launch_param_builder import load_xacro
import xacro
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join, dirname
from pathlib import Path
import os
from launch.actions import ExecuteProcess

from launch.actions import TimerAction


def generate_launch_description():

    # Set up Gazebo resource path

    # This is the package where world models are stored.
    world_package_name = "my_worlds"
    world_file = "duckieworld.sdf"
    # world_file = "eco_disaster_config1.sdf"

    world_share_path = get_package_share_directory(world_package_name)

    # This is the package where robot is stored.
    robot_package_name = "my_arm_description"
    robot_share_path = get_package_share_directory(robot_package_name)

    resources_path = f'{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}:{world_share_path}:{dirname(robot_share_path)}'

    # Start a simulation with the chosen world
    world_uri = join(world_share_path, "worlds", world_file)

    gazebo_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_uri],
        additional_env={
            "GZ_SIM_RESOURCE_PATH": resources_path,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": f'{os.environ.get("LD_LIBRARY_PATH", "")}',
        },
        output="screen",
    )

    # Create a robot in the world.
    # Steps:
    # 1. Process a file using the xacro tool to get an xml file containing the robot description.
    # 2. Publish this robot description using a ros topic so all nodes can know about the joints of the robot.
    # 3. Spawn a simulated robot in the gazebo simulation using the published robot description topic.

    # Step 1. Process robot file.
    robot_file = join(robot_share_path, "urdf", "my_arm.urdf.xacro")
    robot_xml = load_xacro(Path(robot_file))

    # robot_xml = xacro.process_file(str(robot_file)).toxml()

    # Step 2. Publish robot file to ros topic /robot_description & static joint positions to /tf
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_xml, "use_sim_time": True}],
    )

    # Step 3. Spawn a robot in gazebo by listening to the published topic.
    robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "/robot_description",
            "-x",
            "0.2",
            "-y",
            "0.8",
            "-z",
            "0.40",
            "--ros-args",
            "--log-level",
            "debug",
        ],
        name="spawn_robot",
        output="both",
    )

    # Gazebo Bridge: This brings data (sensors/clock) out of gazebo into ROS.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            # '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # '/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/realsense/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output="screen",
    )

    # Step 5: Enable the ros2 controllers
    start_controllers = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "soarm100_controller"],
                output="screen",
            )
        ],
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Launch RViz
    rviz_config = join(
        robot_share_path, "config", "my_arm.rviz"
    )  # Adjust if you have a config file

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            gazebo_sim,
            robot_state_publisher,
            bridge,
            robot,
            joint_state_publisher_gui,
            rviz_node,
            start_controllers,
        ]
    )
