from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_param_builder import load_xacro
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join 
from launch.substitutions import Command
from pathlib import Path
from launch.actions import TimerAction
import os

def generate_launch_description():

    package_name = "my_worlds"

    # Set up Gazebo resource path to include model directory
    world_share_dir = get_package_share_directory(package_name)

    # This ensures Gazebo Sim can find your models (e.g., model://duckie)
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=world_share_dir
    )

    # Optional: Also add to SDF_PATH (used by sdformat_urdf and others)
    set_sdf_path = SetEnvironmentVariable(
        name='SDF_PATH',
        value=world_share_dir
    )

    # Start a simulation with the chosen world
    # world_file = "duckieworld.sdf"
    world_file = "eco_disaster_config1.sdf"
    
    world_uri = join(world_share_dir, "worlds", world_file)
        
    gazebo_launch_path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    
    gazebo_sim = IncludeLaunchDescription(gazebo_launch_path,
                                          launch_arguments=[("gz_args", '-r '+  world_uri)])

    """"
    # Create a robot in the world.
    # Steps: 
    # 1. Process a file using the xacro tool to get an xml file containing the robot description.
    # 2. Publish this robot description using a ros topic so all nodes can know about the joints of the robot. 
    # 3. Spawn a simulated robot in the gazebo simulation using the published robot description topic. 

    # Step 1. Process robot file. 
    robot_file = join(get_package_share_directory("krytn"), "robot_description","krytn.urdf.xacro")
    robot_xml = load_xacro(Path(robot_file))

    #Step 2. Publish robot file to ros topic /robot_description & static joint positions to /tf
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description':robot_xml, 
                     'use_sim_time':True}],
    )

    # Step 3. Spawn a robot in gazebo by listening to the published topic.
    robot = Node(
        package='ros_gz_sim',
        executable="create",
        arguments=[
            "-topic", "/robot_description", 
            "-z", "0.5",
        ],
        name="spawn_robot",
        output="both"
    )

    # Gazebo Bridge: This brings data (sensors/clock) out of gazebo into ROS.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   '/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/realsense/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   ],
        output='screen'
        )

    
    # A gui tool for easy tele-operation.
    robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
    )

    # Step 5: Enable the ros2 controllers
    start_controllers  = TimerAction(
        period=10.0,
        actions=[
        Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=['joint_state_broadcaster', 'diff_drive_base_controller'],
                    output="screen",
                )
            ]
        )

    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper.py",
        remappings=[("/cmd_vel_in", "/cmd_vel"),
                       ("/cmd_vel_out",  "/diff_drive_base_controller/cmd_vel")],
        parameters=[{"use_sim_time","True"}],
        output="screen"
    )  

    # Fix Frames while we wait for merged changes to make their way into released packages: 
    # https://github.com/gazebosim/gz-sensors/pull/446/commits/277d3946be832c14391b6feb6971e243f1968486 
    # This fix allows us to specify the frame for the sensor rather than create a fixed transform here. 
    static_pub = Node(package="tf2_ros", 
                      executable="static_transform_publisher",
                      arguments=["0","0","0","0","0","0", "lidar_2d_link", "krytn/base_footprint/lidar_2d_v1", ])
    
    """
    return LaunchDescription([
                            set_gz_resource_path,
                            set_sdf_path,
                            gazebo_sim,
                            # bridge, 
                            # robot, 
                            # twist_stamper,
                            # robot_steering, 
                            # robot_state_publisher,
                            # start_controllers, 
                            # static_pub,
                            ])