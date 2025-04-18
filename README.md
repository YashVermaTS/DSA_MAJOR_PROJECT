# Pacman Maze Solver

A C++ application that generates random mazes and solves them using different graph algorithms to find the optimal path. This project demonstrates and compares the efficiency of various pathfinding algorithms through visualization.

## Overview

The Pacman Maze Solver generates random 20×20 (or configurable size) mazes with multiple possible paths to the destination. It then solves these mazes using different graph traversal algorithms, visualizes the paths, and provides performance metrics for comparison.

## Features

- **Random maze generation** with configurable dimensions
- **Multiple path generation** (2-4 paths) to enable meaningful algorithm comparison
- Implementation of four different pathfinding algorithms:
  - Depth-First Search (DFS)
  - Breadth-First Search (BFS)
  - A* Search
  - Dijkstra's Algorithm
- **Visualization** of maze and solution paths using SFML
- **Performance metrics** for each algorithm:
  - Execution time
  - Path length
  - Number of nodes visited

## Project Structure

The project consists of the following files:

- `maze.h`: Defines the Maze class and Cell structure
- `maze.cpp`: Implements maze generation and path creation algorithms
- `maze_solver.h`: Defines the MazeSolver class and Point structure
- `MazeSolver.cpp`: Implements the pathfinding algorithms (DFS, BFS, A*, Dijkstra)
- `MazeVisualizer.h`: Defines the visualization interface
- `MazeVisualizer.cpp`: Implements the maze and path visualization using SFML
- `main.cpp`: Contains the main program logic and algorithm comparison functionality

## Requirements

- C++17 compatible compiler
- SFML 2.5 or higher
- CMake 3.10 or higher

## Building the Project

1. Clone the repository

2. Create a build directory:

    ```bash
    mkdir build
    cd build
    ```

3. Configure with CMake:

    ```bash
    cmake ..
    ```

4. Build the project:

    ```bash
    make
    ```

## Running the Application

The application can be run in two modes:

### Visualization Mode (default)

```bash
./pacman_maze_solver
```
### comparison Mode 
``` bash
./pacman_maze_solver compare [num_trials]
```

## Implementation Details

### Maze Generation

The maze is generated using a randomized depth-first search algorithm (recursive backtracker), which creates a perfect maze with exactly one path between any two points. Additional paths are then added by randomly removing walls to create multiple possible routes to the destination.

### Pathfinding Algorithms

All algorithms start at the top-left corner (0,0) and aim to reach the bottom-right corner (width-1, height-1):

- **DFS**: Uses a stack to explore as far as possible along each branch before backtracking
- **BFS**: Uses a queue to explore all neighbors at the present depth before moving to nodes at the next depth level
- **A\***: Uses a priority queue with a Manhattan distance heuristic to guide the search
- **Dijkstra**: Similar to A* but without a heuristic, finding the shortest path in terms of steps

### Visualization

The maze and paths are visualized using SFML:

- Walls are drawn as black lines
- The start point is marked in blue
- The goal point is marked in red
- The solution path is highlighted in green

## Future Enhancements

Potential improvements for the project include:

- Adding weighted paths to better demonstrate the advantages of A* and Dijkstra
- Implementing additional maze generation algorithms
- Adding step-by-step visualization of the pathfinding process
- adding adversaries
