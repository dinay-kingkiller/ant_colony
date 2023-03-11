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

/// \brief This ROS node tracks pheromones and helps ants choose their path.
/// 
/// This node is part of an implementation of the Ant System algorithm using ROS.
/// M. Dorigo, V. Maniezzo, et A. Colorni, Ant system: optimization by a colony of
/// cooperating agents, IEEE Transactions on Systems, Man, and Cybernetics--Part
/// B, volume 26, numéro 1, pages 29-41, 1996.
/// https://www.cs.unibo.it/babaoglu/courses/cas05-06/tutorials/Ant_Colony_Optimization.pdf
///
/// This node interacts with ROS in two ways:
/// 1. This node subscribes to Pheromone* msgs to update the pheromone map.
/// 2. This node provides a direction service based on the pheromone map.

#include <cmath>
#include <ctime>
#include <set>
#include <vector>

#include "ros/ros.h"
#include "ant_colony/PheromoneEdge.h"
#include "ant_colony/PheromonePath.h"
#include "ant_colony/PheromoneMap.h"
#include "ant_colony/Directions.h"
#include "ant_colony/graph.h"

// Parameters
int VertexCount;
float size_x;
float size_y;
float DistancePower;
float EvaporationPower;
float PheromonePower;
float RewardPower;

// Map description
std::vector<float> x_coordinates;
std::vector<float> y_coordinates;
std::vector<std::vector<float>> distances; /// adjacency matrix
std::vector<std::vector<float>> pheromones;
std::vector<std::vector<float>> desirability; /// distances[i][j]**-DistancePower

// Flattened Map (for plotting message)
std::vector<float> from_x;
std::vector<float> from_y;
std::vector<float> to_x;
std::vector<float> to_y;
ant_colony::PheromoneMap map_msg; // TODO: change size from v_c**2 >> (v_c-1)!

int counter; // for seeding srand

/// \brief This function places pheromones along all valid edges.
///
/// This is run once at the start to put pheromones in an initial state.
void SetupPheromones() {
  desirability.resize(VertexCount, std::vector<float>(VertexCount, 0.0));
  pheromones.resize(VertexCount, std::vector<float>(VertexCount, 0.0));
  for (int i = 0; i < VertexCount; ++i) {
    for (int j = 0; j < VertexCount; ++j) {
      if (distances[i][j]!=0) {
	desirability[i][j] = pow(distances[i][j], -DistancePower);
	pheromones[i][j] = 1.0;
      }
    }
  }
}

void SetupMsg() {
  map_msg.from_x.resize(VertexCount*VertexCount, 0.0);
  map_msg.from_y.resize(VertexCount*VertexCount, 0.0);
  map_msg.to_x.resize(VertexCount*VertexCount, 0.0);
  map_msg.to_y.resize(VertexCount*VertexCount, 0.0);
  map_msg.strength.resize(VertexCount*VertexCount, 0.0);
  for (int i = 0; i < VertexCount; ++i) {
    for (int j = 0; j < VertexCount; ++j) {
      // Fill flat vector. Filter later in visualizer
      map_msg.from_x[i*VertexCount+j] = x_coordinates[i];
      map_msg.from_y[i*VertexCount+j] = y_coordinates[i];
      map_msg.to_x[i*VertexCount+j] = x_coordinates[j];
      map_msg.to_y[i*VertexCount+j] = y_coordinates[j];
      map_msg.strength[i*VertexCount+j] = pheromones[i][j];
    }
  }
}

void UpdateMsg() {
  for (int i = 0; i < VertexCount; ++i) {
    for (int j = 0; j < VertexCount; ++j) {
      map_msg.strength[i*VertexCount+j] = pheromones[i][j];
    }
  }
}

/// \brief This function adds pheromones along one edge.
///
///
void AddEdgePheromones(const ant_colony::PheromoneEdge::ConstPtr& msg) {
  pheromones[msg->from_vertex][msg->to_vertex] += msg->deposit;
  pheromones[msg->to_vertex][msg->from_vertex] += msg->deposit;
}

/// \brief
void AddPathPheromones(const ant_colony::PheromonePath::ConstPtr& msg) {
  std::set<std::set<int>> edges;
  std::set<std::set<int>>::iterator e_itr;

  // Convert tour to a SET of edges.
  for (int i = 1; i < msg->tour.size(); ++i) {
    edges.insert({msg->tour[i-1], msg->tour[i]});
  }

  // Add pheromones.
  for (e_itr = edges.begin(); e_itr != edges.end(); ++e_itr) {
    std::set<int> edge = *e_itr;
    std::set<int>::iterator from_vertex = edge.begin();
    std::set<int>::iterator to_vertex = edge.end();
    pheromones[*from_vertex][*to_vertex] += msg->deposit;
    pheromones[*to_vertex][*from_vertex] += msg->deposit;
  }

}

/// \brief Evaporate pheromones.
void UpdatePheromones() {
  for (int i = 0; i < VertexCount; ++i) {
    for (int j = 0; j < VertexCount; ++j) {
      pheromones[i][j] = (1-EvaporationPower) * pheromones[i][j];
    }	  
  }
}


/// \brief ROS Service provided to ants. They ask where they should go 
bool ChoosePath(ant_colony::Directions::Request &req,
		 ant_colony::Directions::Response &res) {
  srand(time(0)+counter);
  counter += rand();
  int start = req.from_here;
  std::vector<int> skip_vertices = req.skip_here;
  double attraction;
  double attraction_ttl = 0.0;
  double desirability_ttl = 0.0;
  double sum;
  double choice = rand() * 1.0 / RAND_MAX;
  bool found;

  if (skip_vertices.size() == VertexCount) {
    // Go home if all of the vertices have been visited.
    res.go_here = 0;
    res.travel_time = distances[start][0];
    return true;
  }

  // Generate attraction and desirability sums.
  for (int i = 0; i < VertexCount; ++i){ 
    found = false;
    for (int j: skip_vertices) {
      if (i==j) {
	found = true;
	break;
      }
    }
    if (!found) {
      attraction = std::pow(pheromones[start][i], PheromonePower) * desirability[start][i];
      attraction_ttl += attraction;
      desirability_ttl += desirability[start][i];
    }
  }

  // Choose based on attraction
  sum = 0.0;
  for (int i = 0; i < VertexCount; ++i) {
    if (attraction_ttl < 0.001) {break;}
    found = false;
    for (int j: skip_vertices) {
      if (i==j) {
	found = true;
	break;
      }
    }
    if (found) {
      // Skip if vertex is in skip_vertices;
      continue;
    }
    attraction = std::pow(pheromones[start][i], PheromonePower) * desirability[start][i];
    if (attraction / attraction_ttl < 0.001) {
      continue;
    }
    sum += attraction;
    if (sum / attraction_ttl + 0.001 > choice) {
      res.go_here = i;
      res.travel_time = distances[start][i];
      return true;
    }
  }

  // Use desirability if there is only faint pheromones left.
  ROS_WARN("Not enough pheromones. Using desirability.");
  for (int i = 0; i < VertexCount; ++i) {
    found = false;
    for (int j: skip_vertices) {
      if (i==j) {
	found = true;
	break;
      }
    }
  }
  sum = 0.0;
  for (int i = 0; i < VertexCount; ++i) {
    if (desirability_ttl < 0.001) {break;}
    found = false;
    for (int j: skip_vertices) {
      if (i==j) {
	found = true;
	break;
      }
    }
    if (found) {
      // Skip if vertex is in skip_vertices;
      continue;
    }
    if (desirability[start][i] / desirability_ttl < 0.001) {
      continue;
    }
    sum += desirability[start][i];
    if (sum / desirability_ttl > choice) {
      res.go_here = i;
      res.travel_time = distances[start][i];
      return true;
    }
  }

  // Dump info if no best path.
  for (int i = 0; i < VertexCount; ++i) {
    found = false;
    for (int j: skip_vertices) {
      if (i==j) {
	found = true;
	break;
      }
    }
    ROS_INFO_STREAM("from_vertex: "<<start
		    <<", to_vertex: "<<i
		    <<", Skipped: "<<found
		    <<", Distance: "<<distances[start][i]
		    <<", Opp: "<<distances[i][start]
		    <<", attraction: "<<pheromones[start][i]
		    <<", desirability: "<<desirability[start][i]);
    ROS_INFO_STREAM("From: "<<x_coordinates[start]<<", "<<y_coordinates[start]
		    <<"; To: "<<x_coordinates[i]<<", "<<y_coordinates[i]);
  }
  ROS_ERROR("Could not find best path.");
  return false;
}


/// When this node is launched it will generate a randomized map for ant travel.
/// This map will track ant pheromones for the direction service. Every second,
/// the pheromones will decay according to the EvaporationPower parameter.
int main(int argc, char **argv) {
  srand(time(NULL));

  // Start up map node.
  ros::init(argc, argv, "map");
  ros::NodeHandle nh;
  ros::Publisher plotter = nh.advertise<ant_colony::PheromoneMap>("map_pheromones", 1000);
  ros::Subscriber scent_path = nh.subscribe("edge_pheromones", 1000, AddEdgePheromones);
  ros::Subscriber scent_edge = nh.subscribe("path_pheromones", 1000, AddPathPheromones);
  ros::ServiceServer scent_choice = nh.advertiseService("directions", ChoosePath);
  ros::Rate loop_rate(1);

  // Set global parameters.
  nh.getParam("VertexCount", VertexCount);
  nh.getParam("size_x", size_x);
  nh.getParam("size_y", size_y);
  nh.getParam("DistancePower", DistancePower);
  nh.getParam("EvaporationPower", EvaporationPower);
  nh.getParam("PheromonePower", PheromonePower);

  // Setup Graph.
  std::string GraphType;
  nh.getParam("GraphType", GraphType);
  if (GraphType == "Complete") {
    GenCompleteGraph(x_coordinates, y_coordinates, distances, VertexCount, size_x, size_y);
  }
  else if (GraphType == "Incomplete") {
    GenIncompleteGraph(x_coordinates, y_coordinates, distances, VertexCount, size_x, size_y);
  }
  else {ROS_ERROR("GraphType not set.");}

  // Track Pheromones.
  SetupPheromones();
  SetupMsg();

  while (ros::ok()) {
    UpdatePheromones();
    UpdateMsg();
    plotter.publish(map_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
