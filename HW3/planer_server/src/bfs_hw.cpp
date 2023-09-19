#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <set>
#include <string> // Include the string library for std::string
#include <algorithm>

struct Point {
    double x, y, z;
    Point() : x(0.0), y(0.0), z(0.0) {} // Default constructor
    Point(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
};

std::unordered_map<std::string, Point> graph_nodes = {
    {"A", Point(0.0, 0.0, 0.0)},
    {"B", Point(1.0, 1.0, 0.0)},
    {"C", Point(2.0, 2.0, 0.0)},
    {"D", Point(3.0, 1.0, 0.0)},
    {"E", Point(4.0, 0.0, 0.0)},
    {"F", Point(2.0, -1.0, 0.0)},
};

std::unordered_map<std::string, std::vector<std::string>> graph_edges = {
    {"A", {"B", "F"}},
    {"B", {"A", "C", "D"}},
    {"C", {"B", "E"}},
    {"D", {"B", "E"}},
    {"E", {"C", "D", "F"}},
    {"F", {"A", "E"}},
};

std::vector<std::string> bfs(const std::string& start, const std::string& end) {
    std::queue<std::string> queue;
    std::unordered_map<std::string, std::string> parent;
    std::set<std::string> visited;

    queue.push(start);
    visited.insert(start);

    while (!queue.empty()) {
        std::string current = queue.front();
        queue.pop();

        if (current == end) {
            // Reconstruct the path
            std::vector<std::string> path;
            std::string node = end;
            while (node != start) {
                path.push_back(node);
                node = parent[node];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const std::string& neighbor : graph_edges[current]) {
            if (visited.find(neighbor) == visited.end()) {
                queue.push(neighbor);
                visited.insert(neighbor);
                parent[neighbor] = current;
            }
        }
    }

    // No path found
    return std::vector<std::string>();
}

// int main() {
//     std::string start_node = "A";
//     std::string end_node = "E";

//     std::vector<std::string> path = bfs(start_node, end_node);

//     if (!path.empty()) {
//         std::cout << "Path from " << start_node << " to " << end_node << ": ";
//         for (const std::string& node : path) {
//             std::cout << node << " ";
//         }
//         std::cout << std::endl;
//     } else {
//         std::cout << "No path from " << start_node << " to " << end_node << " found." << std::endl;
//     }

//     return 0;
// }