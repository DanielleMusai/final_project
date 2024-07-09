# FINAL PROJECT-  Drone Navigation Simulation

## Authorers
Zeev Fischer: 318960242   
Eden Mor: 316160332   
Danielle Musai: 206684755 

## Project Overview -Extension of assignment number 1
This project simulates a drone navigating through a 2D environment, avoiding obstacles and returning to its starting point. The simulation uses Pygame for visualization and implements basic collision detection and pathfinding algorithms.
There are 3 parts to the mapping:
1. The mapping from the first assignment.   
2. When we return to the start, it can be by accident or if the battery is less than 50%, we recall the drone back.   
3. We simulate a second drone that learns from the first mapping. Now we navigate to points we placed along the map that indicate places that are yet to be mapped. When there are no more points, we end the program.   

## Features
- 2D drone movement in a pixel-based environment
- Obstacle avoidance
- Battery life simulation
- Return-to-home functionality
- Real-time visualization of drone path
  
## Requirements
- Python 3.x
- Pygame

## Key Components

### 1. Map Loading
- The simulation uses a PNG image as the map.
- White pixels represent open space, black pixels are obstacles.

### 2. Drone Movement
- The drone moves at a constant speed defined by `DRONE_SPEED_PX_PER_SEC`.
- Movement is controlled by the `move_towards_target` function.

### 3. Collision Detection
- `validate_and_adjust_position` function ensures the drone doesn't collide with obstacles.
- A safety buffer around the drone is checked for potential collisions.

### 4. Pathfinding
- The drone initially explores the environment, avoiding obstacles.
- After half the battery life, it attempts to return to its starting point.

### 5. Battery Simulation
- Battery life is simulated with a countdown timer.
- The simulation ends when the battery is depleted and the drone returns home.

## How To Run
**Noat: Basic understanding of running code is required. There are no special downloads needed; however, keep in mind that some workspaces may differ from others.**
1. for this project we used PyCharm you are encouraged to do the same.
2. Open a workspace that can run Python code.
3. Download the repository and insert the main.py file and the maps directory into your Python workspace.
4. In the main.py file, in the function main, there is a variable called "image_path" that you need to update to the correct path to the map in your system. This should be the path to the Maps directory included in this repository.
