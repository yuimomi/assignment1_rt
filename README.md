# Assignment 1: `assignment1_rt`

## Overview
This ROS package, `assignment1_rt`, implements a solution that involves controlling two turtles (`turtle1` and `turtle2`) within the Turtlesim environment. The package includes two nodes:
- **UI (node1)**: Manages user interaction and controls turtle spawning and movement.
- **Distance (node2)**: Monitors the relative distance between the turtles and enforces boundary constraints.

## Features

### UI Node (node1)
- **Spawn New Turtle (`turtle2`)**:
  - Automatically spawns a second turtle in the Turtlesim environment.
- **Interactive Command Interface**:
  - Provides a text-based interface for the user to:
    - Choose which turtle to control (`turtle1` or `turtle2`).
    - Input velocity commands for the selected turtle.
- **Timed Command Execution**:
  - Velocity commands are executed for 1 second, after which the turtle stops. The user can then input new commands.

### Distance Node (node2)
- **Distance Monitoring**:
  - Computes and publishes the distance between `turtle1` and `turtle2` on a dedicated topic.
- **Proximity Control**:
  - Stops a turtle if it gets too close to the other turtle (user-defined threshold).
- **Boundary Enforcement**:
  - Stops a turtle if it moves out of the defined area:
    - `x` or `y` > 10.0
    - `x` or `y` < 1.0

## Installation

### Prerequisites
- ROS Noetic (recommended)
- Turtlesim package:
  ```bash
  sudo apt install ros-noetic-turtlesim
