#pragma once
#include "maze.h"
#include "maze_solver.h"
#include <SFML/Graphics.hpp>

class MazeVisualizer {
public:
    MazeVisualizer(const Maze& maze, int cellSize = 20);
    void draw(const std::vector<Point>& solution = {});
    void run();
    void saveScreenshot(const std::string& filename);

private:
    const Maze& maze;
    int cellSize;
    sf::RenderWindow window;

    void drawLine(float x1, float y1, float x2, float y2);
    void drawCircle(int x, int y, sf::Color color);
};
