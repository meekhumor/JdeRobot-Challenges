- In src folder, `ros2 pkg create --build-type ament_python <my_ros2_pkg> --dependencies rclpy std_msgs`
- Change the setup.py accordingly

1. Any Changes, `colcon build` in root directory
2. `source install/setup.bash`
3. `ros2 run hello publisher`
4. `ros2 run hello subscriber`


1b.

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