// BSD 3-Clause License
//
// Copyright (c) 2023, Dinay Kingkiller
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <cmath>
#include <vector>
#include "ant_colony/graph.h"

void GenerateRandomGraph(std::vector<std::vector<int>>& graph, int vertex_c, int max_weight) {
  // Generates a connected, undirected, simple, weighted graph.
  //   For any two vertices in a connected graph, there is a path to connect them.
  //   An undirected graph is one where the weight of an edge does not depend on the direction
  //     graph[i][j] == graph[j][i]
  //   A graph is simple if two edges cannot share the same vertices AND
  //   the graph does not contain loops: graph[i][i] == 0
  //   The weight of the edge i->j is stored in graph[i][j]
  // std::vector<std::vector<int>>& graph:
  //   The adjacency matrix of the graph. For missing edges: graph[i][j]
  // int vertex_c:
  //   The size of the graph: how many vertices does it contain?
  //   edge_c (the number of edges) is randomized based on vertex_c
  // int max_weight:
  //   The max edge weight.

  graph.resize(vertex_c, std::vector<int>(vertex_c, 0));

  // Generate a random edge count.
  int max_edges = vertex_c * (vertex_c-1) / 2;
  int min_edges = vertex_c - 1;
  int edge_c = rand() % (max_edges-min_edges) + min_edges;

  // Generate a connected graph.
  int vertex;
  for (int i = 1; i < vertex_c; ++i) {
    vertex = rand() % i;
    graph[vertex][i] = rand() % max_weight + 1;
    graph[i][vertex] = graph[vertex][i];
  }
  
  // Generate the remaining edges.
  int location;
  int from;
  int to;
  int remaining; // remaining open locations in adjacency matrix.
  for (int i = 0; i < edge_c-vertex_c; ++i) {
    remaining = vertex_c*vertex_c - 3*vertex_c + 2 - 2*i;
    for (location = rand() % remaining; location < vertex_c*vertex_c; ++location) {
      if (location/vertex_c == location%vertex_c) {
	continue;
      }
      else if (graph[location/vertex_c][location%vertex_c]!=0) {
	continue;
      }
      else {
	from = location/vertex_c;
	to = location%vertex_c;
	graph[from][to] = rand() % max_weight + 1;
	graph[to][from] = graph[from][to];
	break;
      }
    }
  }
}

void GenerateCompleteGraph(std::vector<std::vector<int>>& graph, int vertex_c, int max_weight) {
  // Generates an undirected, simple, weighted, complete graph.
  //   An undirected graph is one where the weight of an edge does not depend on the direction
  //     graph[i][j] == graph[j][i]
  //   A graph is simple if two edges cannot share the same vertices AND
  //   the graph does not contain loops: graph[i][i] == 0
  //   The weight of the edge i->j is stored in graph[i][j]
  //   A graph is complete if every vertex has an edge to every other vertex
  //     graph[i][j] != 0 for all i, j except if i==j.
  // std::vector<std::vector<int>>& graph:
  //   The adjacency matrix of the graph.
  // int vertex_c:
  //   The size of the graph: how many vertices does it contain?
  //   edge_c (the number of edges) is randomized based on vertex_c
  // int max_weight:
  //   The max edge weight.

  graph.resize(vertex_c, std::vector<int>(vertex_c, 0));
  for (int i = 0; i < vertex_c; ++i) {
    for (int j = i+1; j < vertex_c; ++j) {
      graph[i][j] = rand() % max_weight + 1;
      graph[j][i] = graph[i][j];
    }
  }
}

void GenerateCompleteDigraph(std::vector<std::vector<int>>& graph, int vertex_c, int max_weight) {
  // Generates a directed, simple, weighted, complete graph.
  //   A directed graph is one where the weight of an edge might depend on direction.
  //   A graph is simple if two edges cannot share the same vertices AND
  //   the graph does not contain loops: graph[i][i] == 0
  //   The weight of the edge i->j is stored in graph[i][j]
  //   A graph is complete if every vertex has an edge to every other vertex
  //     graph[i][j] != 0 for all i, j except if i==j.
  // std::vector<std::vector<int>>& graph:
  //   The adjacency matrix of the graph.
  // int vertex_c:
  //   The size of the graph: how many vertices does it contain?
  //   edge_c (the number of edges) is randomized based on vertex_c
  // int max_weight:
  //   The max edge weight.

  graph.resize(vertex_c, std::vector<int>(vertex_c, 0));
  for (int i = 0; i < vertex_c; ++i) {
    for (int j = 0; j < vertex_c; ++j) {
      if (i!=j) {
	graph[i][j] = rand() % max_weight + 1;
      }
    }
  }
}

void GenerateFlatGraph(std::vector<int>& coord_x, std::vector<int>& coord_y, std::vector<std::vector<int>>& graph, int vertex_c, int size_x, int size_y) {
  // Generates an easily plotted complete graph.
  // std::vector<int>& coord_x / std::vector<int>& coord_y
  //   randomized coordinates
  // std::vector<std::vector<int>>& graph:
  //   The adjacency matrix of the graph.
  //   The weight of each edge is the quadrature between two points.
  //   (x_1-x_2)^2 + (y_1-y_2)^2
  // int vertex_c:
  //   The size of the graph: how many vertices does it contain?
  //   edge_c (the number of edges) is randomized based on vertex_c
  // int size_x / int size_y:
  //   The dimensions of the map.

  graph.resize(vertex_c, std::vector<int>(vertex_c, 0));
  coord_x.resize(vertex_c, 0);
  coord_y.resize(vertex_c, 0);
  int diff_x;
  int diff_y;
  for (int i = 0; i < vertex_c; ++i) {
    coord_x[i] = rand() % size_x;
    coord_y[i] = rand() % size_y;
    for (int j = 0; j < i; ++j) {
      diff_x = coord_x[i] - coord_x[j];
      diff_y = coord_y[i] - coord_y[j];
      graph[i][j] = diff_x*diff_x + diff_y*diff_y;
      graph[j][i] = graph[i][j];
    }
  }
}
