#include "MazeVisualizer.h"

MazeVisualizer::MazeVisualizer(const Maze& maze, int cellSize)
    : maze(maze), cellSize(cellSize) {
    window.create(sf::VideoMode(maze.width * cellSize, maze.height * cellSize), "Pacman Maze Solver");
}

void MazeVisualizer::draw(const std::vector<Point>& solution) {
    window.clear(sf::Color::White);

    // Draw maze
    for (int y = 0; y < maze.height; ++y) {
        for (int x = 0; x < maze.width; ++x) {
            int index = y * maze.width + x;
            float px = x * cellSize, py = y * cellSize;

            if (maze.grid[index].walls[0]) drawLine(px, py, px + cellSize, py);
            if (maze.grid[index].walls[1]) drawLine(px + cellSize, py, px + cellSize, py + cellSize);
            if (maze.grid[index].walls[2]) drawLine(px, py + cellSize, px + cellSize, py + cellSize);
            if (maze.grid[index].walls[3]) drawLine(px, py, px, py + cellSize);
        }
    }

    // Draw solution
    for (const auto& p : solution) {
        sf::RectangleShape rect(sf::Vector2f(cellSize / 2, cellSize / 2));
        rect.setPosition(p.x * cellSize + cellSize / 4, p.y * cellSize + cellSize / 4);
        rect.setFillColor(sf::Color::Green);
        window.draw(rect);
    }

    // Draw start and goal
    drawCircle(0, 0, sf::Color::Blue);
    drawCircle(maze.width - 1, maze.height - 1, sf::Color::Red);

    window.display();
}

void MazeVisualizer::run() {
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }
    }
}

void MazeVisualizer::saveScreenshot(const std::string& filename) {
    sf::Texture texture;
    texture.create(window.getSize().x, window.getSize().y);
    texture.update(window);
    texture.copyToImage().saveToFile(filename);
}

void MazeVisualizer::drawLine(float x1, float y1, float x2, float y2) {
    sf::Vertex line[] = {
        sf::Vertex(sf::Vector2f(x1, y1), sf::Color::Black),
        sf::Vertex(sf::Vector2f(x2, y2), sf::Color::Black)
    };
    window.draw(line, 2, sf::Lines);
}

void MazeVisualizer::drawCircle(int x, int y, sf::Color color) {
    sf::CircleShape circle(cellSize / 3);
    circle.setPosition(x * cellSize + cellSize / 3, y * cellSize + cellSize / 3);
    circle.setFillColor(color);
    window.draw(circle);
}
