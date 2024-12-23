# Differential Self-Driving Robot (BumperBot)
[![LinkedIn][linkedin-shield]][linkedin-url]
[![Udemy][udemy-shield]][udemy-url]

## Table of Contents

* [About the Project](#about-the-project)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
* [Usage](#usage)
* [Contributing](#contributing)
* [Acknowledgments](#acknowledgments)

<!-- ABOUT THE PROJECT -->  
## About the Project

This repository contains the material for the **Differential Self-Driving Robot (BumperBot)** project. If you are passionate about robotics and want to develop a self-driving robot with ROS 2, this project is for you! It includes:

- **Odometry**: Real-time position and orientation tracking.
- **Control**: Precise motion control for differential drive.
- **Obstacle Avoidance**: Implementing reactive navigation.
- **Autonomous Navigation**: Path planning and goal-oriented movement.

This project is designed to provide hands-on learning and practical experience with ROS 2 and autonomous robotics.

<!-- GETTING STARTED -->
## Getting Started

You can decide whether to build the real robot or experiment with the simulated one in Gazebo. Most of the lessons and code will work the same in both environments.

### Prerequisites

You don't need prior knowledge of ROS 2 or self-driving concepts. However, a basic understanding of programming in **Python** or **C++** is required.

Prepare your PC by:

* Installing Ubuntu 22.04 on your PC or in a virtual machine. [Download Ubuntu 22.04](https://ubuntu.com/download/).
* Installing [ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or [ROS Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html).
* Installing required ROS 2 packages:

```bash
sudo apt-get update && sudo apt-get install -y \
     ros-humble-navigation2 \
     ros-humble-gazebo-ros \
     ros-humble-ros2-control \
     ros-humble-gazebo-ros2-control \
     ros-humble-joint-state-publisher-gui \
     ros-humble-joy \
     ros-humble-joy-teleop \
     ros-humble-tf-transformations
```

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/pepperumo/Differential_self_driving_Robot_bumperbot.git
   cd Differential_self_driving_Robot_bumperbot
   ```

2. Build the workspace:

   ```bash
   colcon build
   source install/setup.bash
   ```

<!-- USAGE -->
## Usage

### Launch the Simulation

Start the Gazebo simulation environment:

```bash
ros2 launch bumperbot_description gazebo.launch.py
```

### Run the Control Node

Launch the node responsible for robot control:

```bash
ros2 run bumperbot_control control_node
```

### Visualize in RViz

Open RViz for real-time visualization:

```bash
ros2 launch bumperbot_visualization rviz.launch.py
```

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open-source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the repository.
2. Create your feature branch (`git checkout -b feature/AmazingFeature`).
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the branch (`git push origin feature/AmazingFeature`).
5. Open a pull request.

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

Special thanks to the creators of the Udemy course [Self-Driving and ROS 2: Learn by Doing](https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-odometry-control/) for inspiration and guidance.

<!-- MARKDOWN LINKS & IMAGES -->
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/pepperumo/
[udemy-shield]: https://img.shields.io/badge/-Udemy-black.svg?style=flat-square&logo=udemy&colorB=555
[udemy-url]: https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-odometry-control/?referralCode=50BCC4E84DB2DB09BFB3
