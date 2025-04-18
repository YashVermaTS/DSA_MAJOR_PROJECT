#include "maze.h"
#include <random>
#include <stack>
#include <vector>
#include <algorithm>

Maze::Maze(int width, int height) : width(width), height(height), grid(width * height) {
    generateMaze();
}

void Maze::createMultiplePaths(int numAdditionalPaths) {
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = 0; i < numAdditionalPaths; ++i) {
        // Select a random wall that's not on the boundary
        int x = gen() % (width - 2) + 1;
        int y = gen() % (height - 2) + 1;
        
        // Randomly choose a direction (0=top, 1=right, 2=bottom, 3=left)
        int direction = gen() % 4;
        
        int index = y * width + x;
        
        if (grid[index].walls[direction]) {
            grid[index].walls[direction] = false;
            
            int neighborIndex;
            if (direction == 0 && y > 0) { // top
                neighborIndex = (y - 1) * width + x;
                grid[neighborIndex].walls[2] = false; // bottom wall of neighbor
            } else if (direction == 1 && x < width - 1) { // right
                neighborIndex = y * width + (x + 1);
                grid[neighborIndex].walls[3] = false; // left wall of neighbor
            } else if (direction == 2 && y < height - 1) { // bottom
                neighborIndex = (y + 1) * width + x;
                grid[neighborIndex].walls[0] = false; // top wall of neighbor
            } else if (direction == 3 && x > 0) { // left
                neighborIndex = y * width + (x - 1);
                grid[neighborIndex].walls[1] = false; // right wall of neighbor
            }
        }
    }
}

// Check if there's a path from start to end
bool Maze::hasPath(int startX, int startY, int endX, int endY) {
    std::vector<bool> visited(width * height, false);
    std::stack<std::pair<int, int>> stack;
    
    stack.push({startX, startY});
    visited[startY * width + startX] = true;
    
    while (!stack.empty()) {
        auto [x, y] = stack.top();
        stack.pop();
        
        if (x == endX && y == endY) {
            return true;
        }
        
        int index = y * width + x;
        
        // Check all four directions
        if (!grid[index].walls[0] && y > 0 && !visited[(y-1) * width + x]) { // top
            stack.push({x, y-1});
            visited[(y-1) * width + x] = true;
        }
        if (!grid[index].walls[1] && x < width-1 && !visited[y * width + x+1]) { // right
            stack.push({x+1, y});
            visited[y * width + x+1] = true;
        }
        if (!grid[index].walls[2] && y < height-1 && !visited[(y+1) * width + x]) { // bottom
            stack.push({x, y+1});
            visited[(y+1) * width + x] = true;
        }
        if (!grid[index].walls[3] && x > 0 && !visited[y * width + x-1]) { // left
            stack.push({x-1, y});
            visited[y * width + x-1] = true;
        }
    }
    
    return false;
}

void Maze::generateMaze() {
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // First generate a perfect maze using recursive backtracker
    std::stack<int> stack;
    int current = 0;
    grid[current].visited = true;
    
    while (true) {
        std::vector<int> neighbors;
        int x = current % width;
        int y = current / width;
        
        // Check neighbors
        if (y > 0 && !grid[current - width].visited) neighbors.push_back(current - width);
        if (x < width - 1 && !grid[current + 1].visited) neighbors.push_back(current + 1);
        if (y < height - 1 && !grid[current + width].visited) neighbors.push_back(current + width);
        if (x > 0 && !grid[current - 1].visited) neighbors.push_back(current - 1);
        
        if (!neighbors.empty()) {
            int next = neighbors[std::uniform_int_distribution<>(0, neighbors.size() - 1)(gen)];
            stack.push(current);
            
            // Remove walls between current and next
            if (next == current - width) { grid[current].walls[0] = false; grid[next].walls[2] = false; }
            if (next == current + 1) { grid[current].walls[1] = false; grid[next].walls[3] = false; }
            if (next == current + width) { grid[current].walls[2] = false; grid[next].walls[0] = false; }
            if (next == current - 1) { grid[current].walls[3] = false; grid[next].walls[1] = false; }
            
            current = next;
            grid[current].visited = true;
        } else if (!stack.empty()) {
            current = stack.top();
            stack.pop();
        } else {
            break;
        }
    }
    for (auto& cell : grid) {
        cell.visited = false;
    }
    
    int additionalPaths = std::uniform_int_distribution<>(2, 5)(gen);
    
    // Make sure we have at least one path before adding more
    if (!hasPath(0, 0, width-1, height-1)) {
        // If no path exists, create a direct path
        int x = 0, y = 0;
        while (x < width-1 || y < height-1) {
            int index = y * width + x;
            if (x < width-1) {
                grid[index].walls[1] = false; // Remove right wall
                grid[index + 1].walls[3] = false; // Remove left wall of next cell
                x++;
            } else if (y < height-1) {
                grid[index].walls[2] = false; // Remove bottom wall
                grid[index + width].walls[0] = false; // Remove top wall of cell below
                y++;
            }
        }
    }
    
    createMultiplePaths(additionalPaths);
    
    if (!hasPath(0, 0, width-1, height-1)) {
        generateMaze(); // Try again
    }
}