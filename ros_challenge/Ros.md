## Solution Overview
This is my solution for the GSoC-2024 JdeRobot ROS2 Challenge.
1. **ROS2 Basics**: Built a publisher-subscriber setup that sends "Hello! ROS2 is fun" and visualized a TurtleBot3 laser scan in `rviz2`.
2. **Navigation2**: Set up a TurtleBot3 to navigate 3 waypoints in Gazebo using the Navigation2 stack, made the map using SLAM

### Video Demo
https://www.youtube.com/watch?v=-bAVaW1WqQQ

### Part 1a: Publisher-Subscriber ("Hello! ROS2 is fun")
- **Publisher**: Wrote a node that pushes "Hello! ROS2 is fun" to the `/topic` topic every 0.5 seconds.
- **Subscriber**: Another node listens on `/topic` and logs whatever it gets to the terminal.
- **Setup**: Created a package called `hello` with two python files `publisher.py` and `subscriber.py`. Built it with `colcon build`, then ran each node with `ros2 run hello publisher` and `ros2 run hello subscriber`.

![Screenshot from 2025-03-18 21-20-48](https://github.com/user-attachments/assets/672e47cb-3394-4b0b-b9c3-bcec05b4470c)


### Part 1b: TurtleBot Laser Scan Visualization
- **Robot**: Used TurtleBot3 (Waffle model) in Gazebo.
- **Laser Scan**: The bot’s laser sensor see's what’s around it, sending data to `/scan`. Opened `rviz2`, added the `/scan` topic as a `LaserScan` display, and set the fixed frame to `odom`.
- **Teleop**: Added teleoperation to drive the bot around manually. Used `ros2 run teleop_twist_keyboard teleop_twist_keyboard` to send velocity commands. 
- **Execution**: Launched it with `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`

![Screenshot from 2025-03-18 21-18-19](https://github.com/user-attachments/assets/a38f964b-bee6-4c53-b106-7790e5fd3e04)

### Part 2: Navigation2 Waypoint Navigation
- **Setup**: TurtleBot3 in Gazebo again, this time with Navigation2 running for mapping and path planning.
- **Waypoints**: Picked 3 spots. In `rviz2`, set the initial pose with "2D Pose Estimate" (where the bot starts), then dropped goals with "Nav2 Goal" for each waypoint.
- **How It Runs**: Navigation2 builds a map with **SLAM**, plans a path, and moves the bot. Gazebo shows the physical movement; `rviz2` overlays the map and route.

![Screenshot from 2025-03-18 21-19-15](https://github.com/user-attachments/assets/a958e8f5-5935-40bf-a149-9db1da4963cf)


## Source Code
https://github.com/meekhumor/JdeRobot-Challenges/new/main/ros_challenge
