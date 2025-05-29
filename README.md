# 🧭 MazeNavBot

This project was developed to demonstrate basic knowledge of **ROS 2 Jazzy**.  
It features a simple maze-like world built in **Gazebo Harmonic**, where a **TurtleBot3** navigates autonomously using a **LiDAR sensor**. The robot moves forward, detects walls, and navigates the environment based on sensor data.

<p align="center">
  <img src="image.png" alt="Maze World" />
</p>

## 🧪 Technologies Used
- ROS 2 Jazzy
- Gazebo Harmonic
- TurtleBot3
- LiDAR Sensor
- Python (control nodes)

## 🚀 Features
The robot:
- Detects walls using the LiDAR sensor.
- Follows walls.
- Navigates through a maze-like environment.

---

# 📦 How to Run the Program

## ✅ Prerequisites

You need to install the TurtleBot3 packages to use the TurtleBot in Gazebo:
- [TurtleBot3 Quick Start Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

> **Note:**  
> The custom world used in this project is located inside the `turtlebot3_simulations` package.  
> This package is included in the repository and must be downloaded to run the simulation correctly.

---

## 🔧 Setup Instructions

1. **Download the source code** and place it inside a ROS 2 workspace.
2. **Build the workspace**:

    ```bash
    colcon build
    ```

3. **Run the simulation**:

    ```bash
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    export TURTLEBOT3_MODEL=waffle
    ros2 launch turtlebot3_gazebo robot_laberinto.launch.py
    ```

4. **In a second terminal**, rotate the robot before starting the wall-following behavior. This helps it detect the closest wall and begin navigation correctly.

    ```bash
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 run move_scan wall_server
    ```

5. **In a third terminal**, call the service to start the initial rotation:

    ```bash
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 service call /find_wall service_pkg/srv/FindWall {}
    ```

    **Result:**

    <p align="center">
      <img src="task_1.gif" alt="Task 1 Result" />
    </p>

    After completing this task, you can stop the program from step 4.

6. **Start Task 2 (wall following)** in the second terminal:

    ```bash
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch move_scan start_following.launch.py
    ```

    **Result:**

    <p align="center">
      <img src="task_2.gif" alt="Task 2 Result" />
    </p>
