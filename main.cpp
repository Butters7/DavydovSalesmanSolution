#include "fcgraph.hpp"
#include <algorithm>
#include <assert.h>
#include "dsu.hpp"
#include <math.h>
#include <limits>
#include <queue>

double dijkstra(FullyConnectedGraph& graph, size_t start_vertex, size_t goal_vertex) {
	assert(start_vertex >= 0 && start_vertex < graph.verticesCount());
    assert(goal_vertex >= 0 && goal_vertex < graph.verticesCount());

	std::vector<double> d(graph.verticesCount(), std::numeric_limits<double>::max());
	std::priority_queue<std::pair<size_t, double>, std::vector<std::pair<size_t, double>>, EdgeComparator> vertices_queue;

	vertices_queue.push({start_vertex, 0});
	d[start_vertex] = 0;

	while (!vertices_queue.empty()) {
		size_t current = vertices_queue.top().first;
		vertices_queue.pop();

		std::vector<std::pair<size_t, double>> next_vertices = graph.getNextVertices(current);

		for (size_t i = 0; i < next_vertices.size(); ++i) {
            if (d[next_vertices[i].first] == std::numeric_limits<double>::max()) {

                d[next_vertices[i].first] = d[current] + next_vertices[i].second;
                vertices_queue.push({next_vertices[i].first, d[next_vertices[i].first]});

            } else if (d[next_vertices[i].first] > d[current] + next_vertices[i].second) {

                d[next_vertices[i].first] = d[current] + next_vertices[i].second;
                vertices_queue.push({next_vertices[i].first, d[next_vertices[i].first]});

            }
        }
	}

	return d[goal_vertex];
}

double kruskal(std::vector<Edge>& edges, size_t vertices_count) {
	DSU dsu(vertices_count);

	double min_weight = 0;

	std::sort(edges.begin(), edges.end());

	for (auto& edge : edges) {
		int v = dsu.find_set(edge.from_);
		int u = dsu.find_set(edge.to_);

		if (v != u) {
			dsu.union_set(v, u);
			min_weight += edge.weight_;
		}
	}

	return min_weight;
}

void testLogic() {
	{
		std::vector<Edge> edges {
			{0, 1, 9}, {0, 2, 2}, {0, 3, 6}, {1, 3, 2},	{1, 6, 4},
			{2, 4, 1}, {2, 3, 3}, {3, 4, 1}, {3, 5, 9}, {3, 6, 7},
			{4, 7, 6}, {5, 7, 5}, {5, 6, 1}, {5, 8, 1}, {6, 8, 5},
			{7, 8, 5},
		};

		assert(kruskal(edges, 9) == 17);

		std::vector<Edge> answer = {
			{2, 4, 1}, {3, 4, 1}, {5, 6, 1}, {5, 8, 1}, {0, 2, 2},
			{1, 3, 2}, {2, 3, 3}, {1, 6, 4}, {5, 7, 5}, {6, 8, 5},
			{7, 8, 5}, {0, 3, 6}, {4, 7, 6}, {3, 6, 7}, {0, 1, 9},
			{3, 5, 9}
		};

		for (size_t i = 0; i < edges.size(); ++i) {
			assert(edges[i].from_ == answer[i].from_);
			assert(edges[i].to_ == answer[i].to_);
			assert(edges[i].weight_ == answer[i].weight_);
		}

		std::cout << "TEST 1 OK" << std::endl;
	}

	{
		std::vector<Edge> edges {
			{1, 2, 1}, {2, 3, 2}, {3, 4, 5}, {4, 1, 4}
		};

		assert(kruskal(edges, 5) == 7);

		std::cout << "TEST 2 OK" << std::endl;
	}

	{
		std::vector<Edge> edges {
			{4, 3, 3046}, {4, 5, 90110}, {5, 1, 57786}, {3, 2, 28280},
			{4, 3, 18010}, {4, 5, 61367}, {4, 1, 18811}, {4, 2, 69898},
			{3, 5, 72518}, {3, 1, 85838}
		};

		assert(kruskal(edges, 6) == 107923);
		std::cout << "TEST 3 OK" << std::endl;
	}
}

void runForYandexContest(std::istream& input, std::ostream& output) {
	int verticies_count = 0, edges_count = 0;

	input >> verticies_count >> edges_count;

	std::vector<Edge> edges(edges_count, Edge(0, 0, 0));

	int from = 0, to = 0, weight = 0;

	for (size_t i = 0; i < edges_count; ++i) {
		input >> from >> to >> weight;
		edges[i] = Edge(from, to, weight);
	}

	output << kruskal(edges, verticies_count + 1) << std::endl;
}

void runForSalesmanTests() {
	FullyConnectedGraph* graph = nullptr;

	for (size_t i = 2; i <= 10; i++) {

		std::vector<double> approximation_ratio;
		double sum_approximation_ratio = 0;
		double average_approximation = 0;
		double mean_square_deviation = 0;

		for (size_t j = 0; j < 10; ++j) {
			graph = new FullyConnectedGraph(i);
			std::vector<Edge> edges = graph->getEdgesOfGraph();

			double dijkstra_path = dijkstra(*graph, 0, i - 1);
			double kruskal_st_path = kruskal(edges, i + 1);

			approximation_ratio.push_back(kruskal_st_path / dijkstra_path);
			sum_approximation_ratio += kruskal_st_path / dijkstra_path;

			delete graph;
		}

		average_approximation = sum_approximation_ratio / 10;
		
		for (size_t j = 0; j < approximation_ratio.size(); ++j) {
			mean_square_deviation += std::pow(approximation_ratio[i] - average_approximation, 2);
		}

		mean_square_deviation /= 10;

		std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
		std::cout << "Для полносвязного графа из " << i << " точек: " << std::endl;
		std::cout << "Среднее приближение (x с чертой): " << average_approximation << std::endl;
		std::cout << "Среднее квадратичное отклонение (сигма): " << mean_square_deviation << std::endl;
		std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
	}
}

int main() {
	// runForYandexContest(std::cin, std::cout);
	runForSalesmanTests();

	return 0;
}
