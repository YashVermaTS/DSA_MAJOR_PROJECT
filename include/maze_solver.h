#pragma once
#include "maze.h"
#include <vector>
#include <unordered_map>
#include <chrono>

struct Point {
    int x, y;
    
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
    
    bool operator!=(const Point& other) const {
        return !(*this == other);
    }
    
};

namespace std {
    template<>
    struct hash<Point> {
        size_t operator()(const Point& p) const {
            return hash<int>()(p.x) ^ hash<int>()(p.y);
        }
    };
}

struct SolveResult {
    std::vector<Point> path;
    int visitedNodes;
    double executionTime;
};

class MazeSolver {
public:
    static SolveResult solveDFS(const Maze& maze);
    static SolveResult solveBFS(const Maze& maze);
    static SolveResult solveAStar(const Maze& maze);
    static SolveResult solveDijkstra(const Maze& maze);

private:
    static Point getNeighbor(const Point& p, int direction);
    static bool isValid(const Maze& maze, const Point& current, const Point& next);
    static int heuristic(const Point& a, const Point& b);
    static std::vector<Point> reconstructPath(const std::unordered_map<Point, Point>& came_from, 
                                             const Point& start, const Point& goal);
};
