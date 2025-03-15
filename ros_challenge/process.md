## Part 1: Introduction to ROS2

### a. ‘Hello! ROS2 is fun’

- In src folder, `ros2 pkg create --build-type ament_python <my_ros2_pkg> --dependencies rclpy std_msgs`
- Change the setup.py accordingly

1. Any Changes, `colcon build` in root directory
2. `source install/setup.bash`
3. `ros2 run hello publisher`
4. `ros2 run hello subscriber`


### b. Launch your robot

1. Launch Simulation:
   `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`

2. Launch RViz2:
   `rviz2`

3. Configure RViz2:
- Set the Fixed Frame:
    - In the left panel, under “Global Options,” set Fixed Frame to odom.
- Add LaserScan display:
    - Click “Add” (bottom left) > “By topic” > Select /scan under LaserScan > “OK.”

4. `ros2 run turtlebot3_teleop teleop_keyboard` in gazebo terminal
   

-----
## Part 2: ROS2 Navigation2

1. Launch Gazebo Simulation
Start the TurtleBot3 simulation in Gazebo:

`export TURTLEBOT3_MODEL=waffle`
`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`

2. Start Localization (AMCL)

`ros2 launch nav2_bringup localization_launch.py use_sim_time:=True map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml`


3. Open RViz & Set Initial Pose

`ros2 launch nav2_bringup rviz_launch.py`

Set Fixed Frame to map.
Use "2D Pose Estimate" to initialize the robot's location on the map.

4. Start Navigation Stack (Nav2)

`ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True`

5. Use Waypoint Navigation

In RViz, click "2D Nav Goal" and set a goal pose on the map.
TurtleBot3 should navigate to the goal using Nav2.

- To kill rviz and gazebo
pkill -9 ros2
pkill -9 gzserver
pkill -9 gzclient
