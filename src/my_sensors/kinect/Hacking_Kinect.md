# Adding Kinect

## First with realsense

Symlink the `sensors` package to our workspace

```bash
$ ln -s /home/mhered/bar_ws/src/bar_examples/sensors /home/mhered/dev_ws/src/
$ cd /home/mhered/dev_ws
$ colcon build --symlink-install
Starting >>> kinect_ros2
...
Starting >>> sensors
...
Finished <<< kinect_ros2 [0.16s]
Finished <<< sensors [0.99s]

Summary: 5 packages finished [1.13s]
```

From `bar_examples/krytn/robot_description/krytn.urdf.xacro` copy into `my_arm_description/urdf/my_arm.urdf.xacro`

```xml
<!-- Import and add realsense -->
<xacro:include filename="$(find sensors)/realsense_d435/realsense.urdf.xacro" />
<xacro:realsense world_link_name="world">
    <origin xyz="0 1.00 1.2" rpy="0 ${pi/4} -${pi/2}" />
</xacro:realsense>
...
<gazebo>
...
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
        <render_engine>ogre2</render_engine>
    </plugin>
</gazebo>
```

`realsense.urdf.xacro` uses `xacro:macro inertial_box` -> add macro definition at the beginning of file instead of having to symlink all of `magni_description`

```xml
    <xacro:macro name="box_inertial" params="x y z mass">
        <inertial>
            <mass value="${mass}" />
            <origin xyz="-0.15 0 0.03" />
            <inertia ixx="${0.0833333 * mass * (y*y + z*z)}" ixy="0.0" ixz="0.0" iyy="${0.0833333 * mass * (x*x + z*z)}" iyz="0.0" izz="${0.0833333 * mass * (x*x + y*y)}" />
        </inertial>
    </xacro:macro>
```

In the launch file uncomment the realsense arguments of the ROS bridge (??):
```python

    # Gazebo Bridge: This brings data (sensors/clock) out of gazebo into ROS.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            # '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/realsense/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output="screen",
    )
```



### Other stuff from here (write properly!!)

**(Also added static publisher and name to node)** 

In `src/my_robot_arm/launch/gazebo.launch.py`:

* add a `name` argument to the `robot` Node (why?)
* add realsense stuff to the bridge (copy and adapt from `src/bar_examples/maci/launch/gazebo.launch.py` ) 
* add the `static_pub` (copy and adapt from `src/bar_examples/krytn/launch/gazebo.launch.py`)  (why?)

```python
	...   
    # Step 3. Spawn a robot in gazebo by listening to the published topic.
    robot = Node(
        package='ros_gz_sim',
        executable="create",
        arguments=[
            "-name", "my_robot_arm.urdf", # <- added
            "-topic", "/robot_description", 
            "-z", "1.0", # was 0.5 but then arm hits floor
        ],
        name="spawn_robot",
        output="both"
    )

    ...

	# Step 6: Gazebo Bridge: This brings data (sensors/clock) out of gazebo into ROS.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image', # <- added
                '/realsense/depth@sensor_msgs/msg/Image[gz.msgs.Image', # <- added
                '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked', # <- added
            ],
        output='screen')    
    
    # Step 6: Publish static transforms (added)
    static_pub = Node(package="tf2_ros", 
                      executable="static_transform_publisher",
                      arguments=["0","0","0","0","0","0", 
                                 "realsense_link", 
                                 "my_robot_arm/realsense_link/realsense_d435", ])
    ...
    
    return LaunchDescription([
		use_sim_time_launch_arg,
        use_rviz_arg,
        robot_state_publisher,
        rviz,
        gazebo,
        robot,
        start_controllers,
        bridge,
        move_group,
        mg_sim_time,
        static_pub, # added
    ])
```

## 

Compile source and launch:

```bash
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch my_arm_description my_arm.launch.py 
```

Gazebo cannot find dae files for realsense  (Fixed with: adding path as before) 

ros2 topic list shows no point cloud... (Fixed with: )

Now there is topic but no data shown.

Add also dependencies:

**[Write properly]**

Eventually, by comparing with `krytn` robot - in which the realsense depth camera was working -  I found three differences:

* the `package.xml` file was missing the line  ` <depend>realsense2_description</depend>` 
* `CMakeLists.txt` was missing `find_package(realsense2_description REQUIRED)` 
* `src/my_robot_arm/urdf/my_robot_arm.urdf.xacro`was missing the sensors plugin inside the `<gazebo>` tag:

```xml
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find my_robot_arm)/config/ros2_control.yaml</parameters>
    </plugin>

    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
      
  </gazebo>
```

