#pragma once

#include <vector>
#include <utility> // for std::pair

struct Cell {
    bool walls[4] = {true, true, true, true}; // top, right, bottom, left
    bool visited = false;
};

class Maze {
public:
    Maze(int width, int height);
    void generateMaze();
    void createMultiplePaths(int numAdditionalPaths);
    bool hasPath(int startX, int startY, int endX, int endY);
    int width, height;
    std::vector<Cell> grid;
};