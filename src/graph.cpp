// BSD 3-Clause License
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
#include <ctime>
#include <vector>
#include "ant_colony/graph.h"

void GenCompleteGraph(std::vector<float>& coord_x, std::vector<float>& coord_y, std::vector<std::vector<float>>& graph, int vertex_c, float size_x, float size_y) {
  graph.resize(vertex_c, std::vector<float>(vertex_c, 0.0));
  srand(time(NULL));
  coord_x.resize(vertex_c, 0.0);
  coord_y.resize(vertex_c, 0.0);
  float diff_x;
  float diff_y;
  for (int i = 0; i < vertex_c; ++i) {
    coord_x[i] = rand() * size_x / RAND_MAX;
    coord_y[i] = rand() * size_y / RAND_MAX;
    for (int j = 0; j < i; ++j) {
      diff_x = coord_x[i] - coord_x[j];
      diff_y = coord_y[i] - coord_y[j];
      graph[i][j] = sqrt(diff_x*diff_x + diff_y*diff_y);
      graph[j][i] = graph[i][j];
    }
  }
}

void GenIncompleteGraph(std::vector<float>& coord_x, std::vector<float>& coord_y, std::vector<std::vector<float>>& graph, int vertex_c, float size_x, float size_y) {
  srand(time(NULL));
  graph.resize(vertex_c, std::vector<float>(vertex_c, 0.0));
  coord_x.resize(vertex_c, 0.0);
  coord_y.resize(vertex_c, 0.0);

  int diff_x;
  int diff_y;
  
  // Generate a random edge count.
  int max_edges = vertex_c * (vertex_c-1) / 2;
  int min_edges = vertex_c - 1;
  int edge_c = rand() % (max_edges-min_edges) + min_edges;

  // Generate vertices.
  for (int i = 0; i < vertex_c; ++i) {
    coord_x[i] = rand() * size_x / RAND_MAX;
    coord_y[i] = rand() * size_y / RAND_MAX;
  }
  
  // Generate a connected graph.
  for (int i = 1; i < vertex_c; ++i) {
    int j = rand() % i;
    diff_x = coord_x[i] - coord_x[j];
    diff_y = coord_y[i] - coord_y[j];
    graph[i][j] = sqrt(diff_x*diff_x + diff_y*diff_y);
    graph[j][i] = graph[j][i];
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
	diff_x = coord_x[from] - coord_x[to];
	diff_y = coord_y[from] - coord_y[to];
	graph[from][to] = sqrt(diff_x*diff_x + diff_y*diff_y);
	graph[to][from] = graph[from][to];
	break;
      }
    }
  }
}
