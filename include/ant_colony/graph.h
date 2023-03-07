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

#ifndef ANT_COLONY_GRAPH_H
#define ANT_COLONY_GRAPH_H

/// \brief Generates an easily plotted complete graph.
/// std::vector<float>& coord_x / std::vector<float>& coord_y
///   randomized coordinates of the generated graph
/// std::vector<std::vector<int>>& graph:
///   The adjacency matrix of the new graph.
///   The weight of each edge is the length between two points.
///   sqrt((x_1-x_2)^2 + (y_1-y_2)^2)
/// int vertex_c:
///   The size of the graph: how many vertices does it contain?
/// float size_x / float size_y:
///   The dimensions of the map.
void GenCompleteGraph(std::vector<float>& coord_x, std::vector<float>& coord_y, std::vector<std::vector<float>>& graph, int vertex_c, float size_x, float size_y);

/// \brief Generates a plottable, incomplete graph.
/// std::vector<float>& coord_x / std::vector<float>& coord_y
///   randomized coordinates of the generated graph
/// std::vector<std::vector<int>>& graph:
///   The adjacency matrix of the new graph.
///   The weight of each edge is the length between two points.
///   sqrt((x_1-x_2)^2 + (y_1-y_2)^2)
/// int vertex_c:
///   The size of the graph: how many vertices does it contain?
///   edge_c (the number of edges) is randomized based on vertex_c
/// float size_x / float size_y:
///   The dimensions of the map.
void GenIncompleteGraph(std::vector<float>& coord_x, std::vector<float>& coord_y, std::vector<std::vector<float>>& graph, int vertex_c, float size_x, float size_y);

#endif
