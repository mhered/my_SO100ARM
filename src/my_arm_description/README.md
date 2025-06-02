# README.md

1. Configure the xacro file to add `ros2_control` support. In `urdf/my_arm.urdf.xacro` add at the end:

*  a `ros2_control` tag that defines a `<hardware>` tag with the `gz_ros2_control` plugin, and `<command_interface>` and `<state_interface>` tags for each joint (using a macro)
* a `<gazebo>` tag to load the `gz_ros2_control-system` plugin with `config/ros2_controllers.yaml`as parameter file

```xml
    <xacro:macro name="ros2_position_controller" params="joint_name">
        <joint name="${joint_name}">
            <command_interface name="position"/>
            <state_interface name="position">
                <param name="initial_value">0.0</param>
            </state_interface>
            <state_interface name="velocity"/>
        </joint>
    </xacro:macro>

    <ros2_control name="my_arm" type="system">
        <hardware>
            <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        </hardware>

        <xacro:ros2_position_controller joint_name="base_to_shoulder_joint" />
        <xacro:ros2_position_controller joint_name="shoulder_to_upper_arm_joint" />
        <xacro:ros2_position_controller joint_name="upper_arm_to_lower_arm_joint" />
        <xacro:ros2_position_controller joint_name="lower_arm_to_wrist_joint" />
        <xacro:ros2_position_controller joint_name="wrist_to_fixed_jaw_joint" />
        <xacro:ros2_position_controller joint_name="fixed_to_moving_jaw_joint" />

    </ros2_control>

    <gazebo>
        <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
            <parameters>$(find my_arm_description)/config/ros2_controllers.yaml</parameters>

        </plugin>
        <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>
    </gazebo>
```

2. create a new `config/ros2_controllers.yaml` file:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100 #Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      joints:
        - base_to_shoulder_joint
        - shoulder_to_upper_arm_joint
        - upper_arm_to_lower_arm_joint
        - lower_arm_to_wrist_joint
        - wrist_to_fixed_jaw_joint
        - fixed_to_moving_jaw_joint
      command_interfaces:
        - position
      state_interfaces:
        - position
        - velocity
    use_sim_time: True
```

3. install the `config` folder in `CMakeLists.txt`

4. in `my_arm.launch.py` add:

```python
 # Step 5: Enable the ros2 controllers
    start_controllers = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "joint_trajectory_controller"],
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
```

build, source, export environment variable with paths and execute:
```bash
$ colcon build --symlink-install
$ source install/setup.bash
$ export GZ_SIM_RESOURCE_PATH=/home/mhered/dev_ws/src/my_worlds:/home/mhered/dev_ws/src/my_arm_description
$ ros2 launch my_arm_description my_arm.launch.py 
```

does not work: ` [Err] [SystemLoader.cc:92] Failed to load system plugin [gz_ros2_control-system] : Could not find shared library.`

also complains again that it cannot find the mesh files

