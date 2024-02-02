#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std;

struct Node {
    std::string name;
    double cost;
    double heuristic;

    bool operator>(const Node& other) const {
        return cost + heuristic > other.cost + other.heuristic;
    }
};

std::unordered_map<std::string, std::unordered_map<std::string, double>> graph;

std::vector<std::string> a_star(const std::string& start, const std::string& end) {
    std::unordered_map<std::string, double> cost_so_far;
    std::unordered_map<std::string, std::string> came_from;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> frontier;

    cost_so_far[start] = 0.0;
    came_from[start] = "";

    frontier.push({start, 0.0, 0.0});

    while (!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();

        if (current.name == end) {
            // Reconstruct path
            std::vector<std::string> path;
            while (current.name != "") {
                path.push_back(current.name);
                current.name = came_from[current.name];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& neighbor : graph[current.name]) {
            double new_cost = cost_so_far[current.name] + neighbor.second;
            if (cost_so_far.find(neighbor.first) == cost_so_far.end() || new_cost < cost_so_far[neighbor.first]) {
                cost_so_far[neighbor.first] = new_cost;
                came_from[neighbor.first] = current.name;
                double heuristic = 0.0; // You can implement your own heuristic function if needed
                frontier.push({neighbor.first, new_cost, heuristic});
            }
        }
    }

    // If no path is found
    return {};
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <json_file_path> <start_node> <end_node>" << std::endl;
        return 1;
    }

    std::ifstream input_file(argv[1]);
    if (!input_file.is_open()) {
        std::cerr << "Error opening file: " << argv[1] << std::endl;
        return 1;
    }

    json graph_json;
    input_file >> graph_json;

    // The JSON structure is supposed to be 
    // {"node1": {"node2": 1.0, "node3": 2.0, ...}, "node2": {"node1": 3.0, "node3": 4.0, ...}, ...}
    graph = graph_json.get<decltype(graph)>();

    std::string start_node = argv[2];
    std::string end_node = argv[3];

    std::vector<std::string> path = a_star(start_node, end_node);

    if (path.empty()) {
        std::cout << "No path found from " << start_node << " to " << end_node << std::endl;
        return 1;
    }

    std::ofstream output_file("solution.txt");
    if (!output_file.is_open()) {
        std::cerr << "Error creating output file." << std::endl;
        return 1;
    }

    // Write the solution path to the output file
    for (const auto& node : path) {
        output_file << node << std::endl;
    }

    std::cout << "Solution written to solution.txt" << std::endl;

    return 0;
}
