#include "maze.h"
#include "maze_solver.h"
#include "MazeVisualizer.h"
#include <iostream>
#include <iomanip>
#include <functional>
#include <numeric>
#include <string>
#include <unordered_map>

void compareAlgorithms(int numTrials = 10);
void visualizeSingleMaze();

int main(int argc, char* argv[]) {
    if (argc > 1 && std::string(argv[1]) == "compare") {
        int trials = 10;
        if (argc > 2) {
            trials = std::stoi(argv[2]);
        }
        compareAlgorithms(trials);
    } else {
        visualizeSingleMaze();
    }
    return 0;
}

void compareAlgorithms(int numTrials) {
    std::vector<std::pair<std::string, std::function<SolveResult(const Maze&)>>> algorithms = {
        {"DFS", MazeSolver::solveDFS},
        {"BFS", MazeSolver::solveBFS},
        {"A*", MazeSolver::solveAStar},
        {"Dijkstra", MazeSolver::solveDijkstra}
    };

    std::unordered_map<std::string, std::vector<double>> times;
    std::unordered_map<std::string, std::vector<int>> pathLengths;
    std::unordered_map<std::string, std::vector<int>> visitedNodes;

    for (int i = 0; i < numTrials; ++i) {
        Maze maze(40, 40);
        for (const auto& [name, algorithm] : algorithms) {
            SolveResult result = algorithm(maze);
            times[name].push_back(result.executionTime);
            pathLengths[name].push_back(result.path.size());
            visitedNodes[name].push_back(result.visitedNodes);
        }
    }

    // Print results
    std::cout << "Algorithm | Avg Time (ms) | Avg Path Length | Avg Nodes Visited\n";
    std::cout << "--------------------------------------------------------\n";
    for (const auto& [name, _] : algorithms) {
        double avgTime = std::accumulate(times[name].begin(), times[name].end(), 0.0) / numTrials;
        double avgPathLength = std::accumulate(pathLengths[name].begin(), pathLengths[name].end(), 0.0) / numTrials;
        double avgVisitedNodes = std::accumulate(visitedNodes[name].begin(), visitedNodes[name].end(), 0.0) / numTrials;
        
        std::cout << std::left << std::setw(10) << name << " | "
                  << std::setw(13) << std::fixed << std::setprecision(2) << avgTime << " | "
                  << std::setw(16) << std::fixed << std::setprecision(2) << avgPathLength << " | "
                  << std::setw(16) << std::fixed << std::setprecision(2) << avgVisitedNodes << "\n";
    }
}

void visualizeSingleMaze() {
    Maze maze(40, 40);
    MazeVisualizer visualizer(maze);
    
    std::cout << "Solving maze with different algorithms...\n";
    
    // Solve with DFS
    SolveResult dfsSolution = MazeSolver::solveDFS(maze);
    std::cout << "DFS: " << dfsSolution.path.size() << " steps, " 
              << dfsSolution.visitedNodes << " nodes visited, " 
              << dfsSolution.executionTime << " ms\n";
    
    // Solve with BFS
    SolveResult bfsSolution = MazeSolver::solveBFS(maze);
    std::cout << "BFS: " << bfsSolution.path.size() << " steps, " 
              << bfsSolution.visitedNodes << " nodes visited, " 
              << bfsSolution.executionTime << " ms\n";
    
    // Solve with A*
    SolveResult astarSolution = MazeSolver::solveAStar(maze);
    std::cout << "A*: " << astarSolution.path.size() << " steps, " 
              << astarSolution.visitedNodes << " nodes visited, " 
              << astarSolution.executionTime << " ms\n";
    
    // Solve with Dijkstra
    SolveResult dijkstraSolution = MazeSolver::solveDijkstra(maze);
    std::cout << "Dijkstra: " << dijkstraSolution.path.size() << " steps, " 
              << dijkstraSolution.visitedNodes << " nodes visited, " 
              << dijkstraSolution.executionTime << " ms\n";
    
    visualizer.draw(astarSolution.path);

    std::cout << "Press any key to exit...\n";
    visualizer.run();
}
