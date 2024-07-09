#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <limits>
#include <queue>
#include <algorithm>

// Graph class to represent the solar system
class Graph {
public:
    // Adjacency list representation of the graph
    std::unordered_map<std::string, std::vector<std::pair<std::string, double>>> adjList;
    // Method to add a node to the graph
    void addNode(const std::string& node) {
        adjList[node] = std::vector<std::pair<std::string, double>>();
    }

    // Method to add an edge with distance between two nodes
    void addEdge(const std::string& src, const std::string& dest, double distance) {
        adjList[src].push_back({dest, distance});
    }

    // Method to get neighbors of a node
    std::vector<std::pair<std::string, double>> getNeighbors(const std::string& node) const {
        if (adjList.find(node) != adjList.end()) {
            return adjList.at(node);
        } else {
            return {};
        }
    }
// private:
};

// Pathfinding class to implement Dijkstra's algorithm
class Pathfinding {
public:
    // Constructor to initialize the graph reference
    Pathfinding(const Graph& graph) : graph(graph) {}

    // Dijkstra's algorithm to find the shortest path
    std::vector<std::string> dijkstra(const std::string& start, const std::string& goal) {
        std::unordered_map<std::string, double> distances;
        std::unordered_map<std::string, std::string> previous;
        std::priority_queue<std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, std::greater<>> priorityQueue;

        // Initialize distances to infinity
        for (const auto& node : graph.adjList) {
            distances[node.first] = std::numeric_limits<double>::infinity();
        }

        // Distance to the start node is zero
        distances[start] = 0;
        priorityQueue.push({0, start});

        while (!priorityQueue.empty()) {
            std::string current = priorityQueue.top().second;
            priorityQueue.pop();

            // If the goal is reached, reconstruct the path
            if (current == goal) {
                std::vector<std::string> path;
                for (std::string at = goal; at != ""; at = previous[at]) {
                    path.push_back(at);
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            // Update distances and previous nodes
            for (const auto& neighbor : graph.getNeighbors(current)) {
                double newDist = distances[current] + neighbor.second;
                if (newDist < distances[neighbor.first]) {
                    distances[neighbor.first] = newDist;
                    previous[neighbor.first] = current;
                    priorityQueue.push({newDist, neighbor.first});
                }
            }
        }

        return {}; // Return empty path if no path is found
    }

private:
    const Graph& graph; // Reference to the graph
};

// Resources class to manage the resources required for the mission
class Resources {
public:
    // Constructor to initialize the fuel
    Resources(double fuel) : fuel(fuel) {}

    // Method to consume fuel
    bool consumeFuel(double amount) {
        if (fuel < amount) return false; // Check if there is enough fuel
        fuel -= amount; // Subtract the consumed fuel
        return true;
    }

    // Method to get the remaining fuel
    double getFuel() const {
        return fuel;
    }

private:
    double fuel; // Amount of fuel
};

// Main function to run the space mission scheduler
int main() {
    Graph solarSystem;
    // Adding nodes (planets) to the graph
    solarSystem.addNode("Earth");
    solarSystem.addNode("Moon");
    solarSystem.addNode("Mars");
    solarSystem.addNode("Phobos");

    // Adding edges (routes) with distances between planets
    solarSystem.addEdge("Earth", "Moon", 1);
    solarSystem.addEdge("Moon", "Mars", 2);
    solarSystem.addEdge("Mars", "Phobos", 1);

    Pathfinding pathfinding(solarSystem);
    Resources resources(3); // Initialize resources with 10 units of fuel

    std::string start, end;
    std::cout << "Enter start planet: ";
    std::cin >> start;
    std::cout << "Enter destination planet: ";
    std::cin >> end;

    std::cout << "Planning mission from " << start << " to " << end << "...\n";
    std::vector<std::string> path = pathfinding.dijkstra(start, end);

    if (!path.empty()) {
        std::cout << "Mission Plan:\n";
        double totalDistance = 0;
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << path[i];
            if (i < path.size() - 1) {
                std::cout << " -> ";
                // Calculate the total distance
                totalDistance += std::find_if(solarSystem.getNeighbors(path[i]).begin(), solarSystem.getNeighbors(path[i]).end(),
                                              [&path, i](const std::pair<std::string, double>& p) { return p.first == path[i + 1]; })->second;
            }
        }
        std::cout << "\nTotal Distance: " << totalDistance << "\n";

        // Check if there is enough fuel for the mission
        if (resources.consumeFuel(totalDistance)) {
            std::cout << "Mission successful! Remaining fuel: " << resources.getFuel() << "\n";
        } else {
            std::cout << "Mission failed due to insufficient fuel.\n";
        }
    } else {
        std::cout << "Failed to plan mission due to pathfinding constraints.\n";
    }

    return 0;
}
