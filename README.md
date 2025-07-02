# 🧭 Custom DWA Local Planner (ROS2 Humble)

A ROS2 Humble package that implements a **Custom Dynamic Window Approach (DWA)** local planner from scratch for **TurtleBot3** in Gazebo.

> 🚀 Developed as part of an assignment to implement a DWA planner without using the Nav2 stack's default DWB controller.

---

## 📦 Repository Structure

custom_dwa_ws/
├── src/

│ └── custom_dwa/

│ ├── custom_dwa/

│ │ ├── dwa_algorithm.py # Core DWA logic

│ │ ├── dwa_planner_node.py # ROS2 Node handling subscriptions, control loop

│ ├── setup.py

│ ├── package.xml

│ └── test/ # Linting and style check scripts


---

## 🛠️ Features

- ✅ Samples velocity commands using dynamic constraints

- ✅ Predicts robot trajectories using motion model

- ✅ Evaluates trajectories with a multi-objective cost function:
  - Obstacle avoidance
  - Goal proximity
  - Path smoothness
  - Speed preference

- ✅ Publishes best `cmd_vel` to control the robot

- ✅ Visualizes trajectories and goal in RViz

- ✅ Integrates with standard ROS2 topics (`/odom`, `/scan`, `/cmd_vel`, `/goal_pose`)


---

## 🧰 Prerequisites

Make sure you have:

- Ubuntu 22.04
- ROS 2 Humble installed
- TurtleBot3 packages
- Gazebo simulator

---

## 🧱 Setting Up `turtlebot3_ws and running custom dwa planner`

1. Install Required Packages

sudo apt update

sudo apt install ros-humble-desktop \

    ros-humble-navigation2 \
    
    ros-humble-nav2-bringup \
    
    ros-humble-gazebo-ros-pkgs

2. Create & Build Workspace

mkdir -p ~/turtlebot3_ws/src

cd ~/turtlebot3_ws/src

# Clone TurtleBot3 packages

git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git

git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

cd ~/turtlebot3_ws

colcon build --symlink-install

3. Add to .bashrc


echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc

echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

source ~/.bashrc

🚀 How to Run the Simulation + Planner

Terminal 1 – Launch Gazebo Simulation

source /opt/ros/humble/setup.bash

source ~/turtlebot3_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


Terminal 2 – Run the DWA Planner

source /opt/ros/humble/setup.bash

source ~/custom_dwa_ws/install/setup.bash

ros2 run custom_dwa dwa_planner_node

3. Set Goal in RViz

    Open RViz (automatically launches with Gazebo)

    Use "2D Nav Goal" tool to set a target position

    Robot will begin navigating using the custom DWA planner

📌 Notes

    /cmd_vel is published by planner

    /odom, /scan, /goal_pose are subscribed

    /dwa_trajectories visualizes the predicted path

    Logs provide info/debug on planner and goal status


🧠 Assignment Compliance Summary

Requirement	Status

Custom DWA logic (not using Nav2’s DWB)	✅

Cost-based trajectory evaluation	✅

Integration with /odom, /scan, /cmd_vel, /goal_pose	✅

RViz visualization using markers	✅

Gazebo-based simulation with TurtleBot3	✅

Informative logs and terminal output	✅

Proper README with full instructions	✅


📃 Maintainer

    Karan Patel

    📧 karanpatelkp1002@gmail.com

    📝 License: Apache 2.0
    
    
    🔗 References

    TurtleBot3 Simulation Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

    Quick Start Guide: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

    DWA Algorithm Paper: https://ieeexplore.ieee.org/document/5399136
