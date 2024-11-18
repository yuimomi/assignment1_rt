# Assignment 1: `assignment1_rt`

## Overview
This ROS package, `assignment1_rt`, implements a solution that involves controlling two turtles (`turtle1` and `turtle2`) within the Turtlesim environment. The package includes two nodes:
- **UI (node1)**: Manages user interaction and controls turtle spawning and movement.
- **Distance (node2)**: Monitors the relative distance between the turtles and enforces boundary constraints.

## Features

### UI Node (node1)
- **Spawn New Turtle (`turtle2`)**
  - Automatically spawns a second turtle in the Turtlesim environment.
- **Interactive Command Interface**
  - Provides a text-based interface for the user to:
    - Choose which turtle to control (`turtle1` or `turtle2`).
    - Input velocity commands for the selected turtle.
- **Timed Command Execution**
  - Velocity commands are executed for 1 second, after which the turtle stops. The user can then input new commands.

### Distance Node (node2)
- **Distance Monitoring**
  - Computes and publishes the distance between `turtle1` and `turtle2` on a dedicated topic.
- **Proximity Control**
  - Stops a turtle if it gets too close to the other turtle (user-defined threshold).
- **Boundary Enforcement**
  - Stops a turtle if it moves out of the defined area:
    - `x` or `y` > 10.0
    - `x` or `y` < 1.0

## Installation

### Prerequisites
- ROS Noetic (recommended)
- Turtlesim package
  ```
  sudo apt install ros-noetic-turtlesim
  ```

### Build the Package
1. Clone the repository into your ROS workspace
  ```
  cd ~/catkin_ws/src
  git clone [repository URL]
  cd ~/catkin_ws
  catkin_make
 ```

2. Source your workspace
  ```
  source devel/setup.bash
```

### How to Run
1. Launch Turtlesim
  ```
  rosrun turtlesim turtlesim_node
  ```
2. Start the Nodes
  - Launch the UI Node (node1)
  ```
  rosrun assignment1_rt UI
  ```
  
  - Launch the Distance Node (node2)
  ```
  rosrun assignment1_rt Distance
  ```

3. Interact with the Interface
   - Use the terminal interface to select a turtle and input velocity commands.

### Topics and Services
#### Topics
- `/turtle1/pose`: Provides the position and orientation of `turtle1`.
- `/turtle2/pose`: Provides the position and orientation of `turtle2`.
- `/cmd_vel`: Accepts velocity commands for the turtles.
- `/distance_topic`: Publishes the relative distance between `turtle1` and `turtle2`.

#### Services
- `/spawn`: Spawns a new turtle.
- `/kill`: Removes a turtle.

### Code Structure
```
assignment1_rt/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── UI.cpp            # Implements the UI node
│   ├── Distance.cpp      # Implements the distance monitoring node
└── README.md
```

### Evaluation
- Code must be pushed to a GitHub repository with a clear commit history.
- README must clearly explain the code, its functionality, and how to run it.
- Bonus points for using both C++ and Python in your implementation.



