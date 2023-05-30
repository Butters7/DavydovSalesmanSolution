#include "fcgraph.hpp"

FullyConnectedGraph::FullyConnectedGraph(size_t verticies_count) : vertices_count_(verticies_count) {
    createGraph();
}

std::vector<std::pair<size_t, double>> FullyConnectedGraph::getNextVertices(size_t vertix) const {
    std::vector<std::pair<size_t, double>> next_vertices;

    for (size_t i = 0; i < edges_.size(); ++i) {
        if (vertix == edges_[i].from_)
            next_vertices.push_back({edges_[i].to_, edges_[i].weight_});
        else if (vertix == edges_[i].to_)
            next_vertices.push_back({edges_[i].from_, edges_[i].weight_});
    }

    return next_vertices;
}

size_t FullyConnectedGraph::verticesCount() const {
    return vertices_count_;
}

std::vector<Edge> FullyConnectedGraph::getEdgesOfGraph() const {
    return edges_;
}

void FullyConnectedGraph::createGraph() {
    std::random_device rd;
    std::mt19937 gen(rd());

    double u = 0, v = 0, s = 0;

    for (size_t i = 0; i < vertices_count_; ++i) {

        for (size_t j = i + 1; j < vertices_count_; ++j) {

            while (s == 0 || s > 1) {
                std::uniform_real_distribution<double> dis(-1.0, 1.0);

                u = dis(gen);
                v = dis(gen);

                s = u * u + v * v;
            }

            edges_.push_back(Edge(i, j, s));
            s = 0;
        }
    }
}

std::ostream &operator<<(std::ostream &out, FullyConnectedGraph &graph) {
    for (size_t i = 0; i < graph.edges_.size(); ++i) {
        out << graph.edges_[i].from_ << " <--> " << graph.edges_[i].to_ << " (" << graph.edges_[i].weight_ << "), " << std::endl;
    }

    return out;
}
