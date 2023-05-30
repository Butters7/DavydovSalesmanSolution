#pragma once

#include <iostream>
#include <random>
#include <vector>

struct EdgeComparator {
    bool operator()(const std::pair<size_t, double>& left, const std::pair<size_t, double>& right) const {
        return left.second > right.second;
    }
};

struct Edge {
	Edge(size_t from, size_t to, double weight) : from_(from), to_(to), weight_(weight) {
	}

	bool operator<(const Edge& edge) const {
		return weight_ < edge.weight_;
	}

	size_t from_;
	size_t to_;
	double weight_;
};

class FullyConnectedGraph {
public:
    FullyConnectedGraph(size_t verticies_count);

    std::vector<std::pair<size_t, double>> getNextVertices(size_t vertix) const;

    size_t verticesCount() const;
    
    std::vector<Edge> getEdgesOfGraph() const;

    friend std::ostream& operator<<(std::ostream& out, FullyConnectedGraph& graph);

private:
    void createGraph();

    size_t vertices_count_;
    std::vector<Edge> edges_;
};
