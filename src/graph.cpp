// Graph Generation
#include <cmath>

void GenerateIncompleteGraph(std::vector<std::vector<float>>& graph, int vertex_c, float max_weight) {
  graph.resize(size, std::vector<float>(vertex_c, 0.0));

  // Generate a random edge count.
  int max_edges = vertex_c * (vertex_c-1) / 2;
  int min_edges = vertex_c - 1;
  int edge_c = rand() % (max_edges-min_edges) + min_edges;

  // Generate a connected graph.
  int vertex;
  for (int i = 1; i < vertex_c; ++i) {
    vertex = rand() % i;
    graph[vertex][i] = rand() % max_weight;
    graph[i][vertex] = graph[vertex][i];
  }

  // Generate the remaining edges.
  int location;
  int from;
  int to;
  int remaining; // remaining open locations in adjacency matrix.
  for (int i = 0; i < edge_c-vertex_c; ++i) {
    remaining = vertex_c*vertex_c - 2*vertex_c + 2 - 2*i;
    location = rand() % remaining;
    for (int j = location; j == 0; --j) {
      if (j/vertex_c == j%vertex_c) ++location;
      if (graph[j/vertex_c][j%vertex_c]!=0) ++location;
    }
    from = location/vertex_c;
    to = location/vertex_c;
    graph[from][to] = rand() % max_weight;
    graph[to][from] = graph[from][to];
  }
}

void GenerateCompleteGraph(std::vector<std::vector<float>>& graph, int vertex_c, float max_weight) {
  graph.resize(size, std::vector<float>(size, 0.0));
  for (int i = 0; i < vertex_c; ++i) {
    for (int j = i+1; j < vertex_c; ++j) {
      graph[i][j] = rand() % max_weight;
      graph[j][i] = graph[i][j];
    }
  }
}

void GenerateCompleteDigraph(std::vector<std::vector<float>>& graph, int vertex_c, float max_weight) {
  graph.resize(size, std::vector<float>(size, 0.0));
  for (int i = 0; i < vertex_c; ++i) {
    for (int j = 0; j < vertex_c; ++j) {
      if (i!=j) {
	graph[i][j] = rand() % max_weight;
      }
    }
  }
}
