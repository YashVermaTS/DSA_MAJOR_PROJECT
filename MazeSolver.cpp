#include "maze_solver.h"
#include <queue>
#include <stack>
#include <cmath>
#include <algorithm>
#include<iostream>
Point MazeSolver::getNeighbor(const Point& p, int direction) {
    switch (direction) {
        case 0: return {p.x, p.y - 1}; // top
        case 1: return {p.x + 1, p.y}; // right
        case 2: return {p.x, p.y + 1}; // bottom
        case 3: return {p.x - 1, p.y}; // left
        default: return p;
    }
}

bool MazeSolver::isValid(const Maze& maze, const Point& current, const Point& next) {
    if (next.x < 0 || next.x >= maze.width || next.y < 0 || next.y >= maze.height) return false;
    int index = current.y * maze.width + current.x;
    if (next.x < current.x && maze.grid[index].walls[3]) return false; // left
    if (next.x > current.x && maze.grid[index].walls[1]) return false; // right
    if (next.y < current.y && maze.grid[index].walls[0]) return false; // top
    if (next.y > current.y && maze.grid[index].walls[2]) return false; // bottom
    return true;
}

int MazeSolver::heuristic(const Point& a, const Point& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

std::vector<Point> MazeSolver::reconstructPath(const std::unordered_map<Point, Point>& came_from, 
                                          const Point& start, const Point& goal) {
    std::vector<Point> path;
    Point current = goal;
    
    // Check if goal is reachable
    if (came_from.find(goal) == came_from.end() && goal != start) {
        return path; // Empty path if goal is not reachable
    }
    
    while (!(current == start)) {
        path.push_back(current);
        auto it = came_from.find(current);
        if (it == came_from.end()) break;
        current = it->second;
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

SolveResult MazeSolver::solveDFS(const Maze& maze) {
    auto start_time = std::chrono::high_resolution_clock::now();
    std::stack<Point> stack;
    std::unordered_map<Point, Point> visited;
    Point start{0, 0}, goal{maze.width - 1, maze.height - 1};

    stack.push(start);
    visited[start] = start;

    while (!stack.empty()) {
        Point current = stack.top();
        stack.pop();

        if (current == goal) break;

        for (int i = 0; i < 4; ++i) {
            Point next = getNeighbor(current, i);
            if (isValid(maze, current, next) && visited.find(next) == visited.end()) {
                stack.push(next);
                visited[next] = current;
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    return {reconstructPath(visited, start, goal), static_cast<int>(visited.size()), duration.count() / 1000.0};
}

SolveResult MazeSolver::solveBFS(const Maze& maze) {
    auto start_time = std::chrono::high_resolution_clock::now();
    std::queue<Point> queue;
    std::unordered_map<Point, Point> visited;
    Point start{0, 0}, goal{maze.width - 1, maze.height - 1};

    queue.push(start);
    visited[start] = start;

    while (!queue.empty()) {
        Point current = queue.front();
        queue.pop();

        if (current == goal) break;

        for (int i = 0; i < 4; ++i) {
            Point next = getNeighbor(current, i);
            if (isValid(maze, current, next) && visited.find(next) == visited.end()) {
                queue.push(next);
                visited[next] = current;
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    return {reconstructPath(visited, start, goal), static_cast<int>(visited.size()), duration.count() / 1000.0};
}

SolveResult MazeSolver::solveAStar(const Maze& maze) {
    auto start_time = std::chrono::high_resolution_clock::now();
    auto cmp = [](const std::pair<int, Point>& a, const std::pair<int, Point>& b) {
        return a.first > b.first;
    };
    std::priority_queue<std::pair<int, Point>, std::vector<std::pair<int, Point>>, decltype(cmp)> pq(cmp);
    std::unordered_map<Point, Point> came_from;
    std::unordered_map<Point, int> g_score;
    Point start{0, 0}, goal{maze.width - 1, maze.height - 1};

    pq.push({heuristic(start, goal), start});
    g_score[start] = 0;

    while (!pq.empty()) {
        Point current = pq.top().second;
        pq.pop();

        if (current == goal) break;

        for (int i = 0; i < 4; ++i) {
            Point next = getNeighbor(current, i);
            if (isValid(maze, current, next)) {
                int tentative_g = g_score[current] + 1;
                if (g_score.find(next) == g_score.end() || tentative_g < g_score[next]) {
                    came_from[next] = current;
                    g_score[next] = tentative_g;
                    int f_score = tentative_g + heuristic(next, goal);
                    pq.push({f_score, next});
                }
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    return {reconstructPath(came_from, start, goal), static_cast<int>(came_from.size()), duration.count() / 1000.0};
}

SolveResult MazeSolver::solveDijkstra(const Maze& maze) {
    auto start_time = std::chrono::high_resolution_clock::now();
    auto cmp = [](const std::pair<int, Point>& a, const std::pair<int, Point>& b) {
        return a.first > b.first;
    };
    std::priority_queue<std::pair<int, Point>, std::vector<std::pair<int, Point>>, decltype(cmp)> pq(cmp);
    std::unordered_map<Point, Point> came_from;
    std::unordered_map<Point, int> distance;
    Point start{0, 0}, goal{maze.width - 1, maze.height - 1};

    pq.push({0, start});
    distance[start] = 0;

    while (!pq.empty()) {
        Point current = pq.top().second;
        int current_dist = pq.top().first;
        pq.pop();

        if (current == goal) break;

        // Skip if we've found a better path already
        if (current_dist > distance[current]) continue;

        for (int i = 0; i < 4; ++i) {
            Point next = getNeighbor(current, i);
            if (isValid(maze, current, next)) {
                int tentative_dist = distance[current] + 1;
                if (distance.find(next) == distance.end() || tentative_dist < distance[next]) {
                    came_from[next] = current;
                    distance[next] = tentative_dist;
                    pq.push({tentative_dist, next});
                }
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    return {reconstructPath(came_from, start, goal), static_cast<int>(came_from.size()), duration.count() / 1000.0};
}
