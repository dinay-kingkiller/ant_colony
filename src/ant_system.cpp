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

//
std::vector<float> x_coordinates;
std::vector<float> y_coordinates;
std::vector<std::vector<float>> distances; /// adjacency matrix
std::vector<float> pheromones;
std::vector<std::vector<float>> desirability; /// distances[i][j]**-DistancePower

void SetupPheromones() {
  desirability.resize(VertexCount, std::vector<float>(VertexCount, 0.0));
  pheromones.resize(VertexCount*VertexCount, 0.0);
  for (int i = 0; i < VertexCount; ++i) {
    for (int j = 0; j < VertexCount; ++j) {
      if (distances[i][j]!=0) {
	desirability[i][j] = pow(distances[i][j], -DistancePower);
	pheromones[i*VertexCount+j] = 1.0;
      }
    }
  }
}

void AddEdgePheromones(const ant_colony::PheromoneEdge::ConstPtr& msg) {
  float max;
  int pheromone_i = msg->from_vertex * VertexCount + msg->to_vertex;
  int pheromone_j = msg->to_vertex * VertexCount + msg->from_vertex;
  pheromones[pheromone_i] += msg->deposit;
  pheromones[pheromone_j] += msg->deposit;
  max = pheromones[pheromone_i];
  // TODO: Verify pheromones don't go up if not deposited.
  // i.e. new deposits should always be the max value.
  for (int i = 0; i < VertexCount*VertexCount; ++i) {
    if (pheromones[i] < .001) {
      pheromones[i] = 0.0;
    }
    else if (pheromones[i] > max) {
      ROS_WARN_STREAM("Possible max value"<<pheromones[i]/max);
      ROS_WARN_STREAM("i: "<<i/VertexCount<<" j: "<<i%VertexCount);
      pheromones[i] = 1.0;
    }
    else {
      pheromones[i] = pheromones[i] / max;
    }
  }
}

void AddPathPheromones(const ant_colony::PheromonePath::ConstPtr& msg) {
  float max;
  int from_vertex;
  int to_vertex;
  std::set<std::set<int>> edges;
  std::set<std::set<int>>::iterator e_itr;

  // Filter edges.
  for (int i = 1; i < msg->tour.size(); ++i) {
    edges.insert({msg->tour[i-1], msg->tour[i]});
  }

  // Add pheromones.
  max = 0.0;
  for (e_itr = edges.begin(); e_itr != edges.end(); ++e_itr) {
    std::set<int> edge = *e_itr;
    std::set<int>::iterator from_vertex = edge.begin();
    std::set<int>::iterator to_vertex = edge.end();
    pheromones[*from_vertex * VertexCount + *to_vertex] += msg->deposit;
    pheromones[*to_vertex * VertexCount + *from_vertex] += msg->deposit;
    if (pheromones[*from_vertex * VertexCount + *to_vertex] > max) {
      max = pheromones[*from_vertex * VertexCount + *to_vertex];
    }
  }

  // Normalize pheromones.
  // TODO: Verify pheromones don't go up if not deposited.
  // i.e. new deposits should always be the max value.
  for (int i = 0; i < VertexCount*VertexCount; ++i) {
    if (pheromones[i] < .001) {
      pheromones[i] = 0.0;
    }
    else if (pheromones[i] + .001 > max) {
      ROS_WARN_STREAM("Possible max value"<<pheromones[i]/max);
      ROS_WARN_STREAM("i: "<<i/VertexCount<<" j: "<<i%VertexCount);
      pheromones[i] = 1.0;
    }
    else {
      pheromones[i] = pheromones[i] / max;
    }
  }
}

void UpdatePheromones() {
  for (int i = 0; i < VertexCount*VertexCount; ++i) {
    pheromones[i] = (1-EvaporationPower) * pheromones[i];
  }
}

bool ChoosePath(ant_colony::Directions::Request &req,
		 ant_colony::Directions::Response &res) {
  ROS_INFO_STREAM("Inside Choice");
  int start = req.from_here;
  double attraction;
  double attraction_ttl = 0.0;
  double desirability_ttl = 0.0;
  double sum;
  double choice = rand() * 1.0 / RAND_MAX;

  // Generate attraction and desirability sums.
  for (int i = 0; i < VertexCount; ++i) {
    attraction = std::pow(pheromones[start*VertexCount + i], PheromonePower)
      * desirability[start][i];
    attraction_ttl += attraction;
    desirability_ttl += desirability[start][i];
  }

  // Choose based on attraction
  sum = 0.0;
  for (int i = 0; i < VertexCount; ++i) {
    attraction = std::pow(pheromones[start*VertexCount + i], PheromonePower)
      * desirability[start][i];
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
  sum = 0.0;
  for (int i = 0; i < VertexCount; ++i) {
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
  while (ros::ok()) {
    UpdatePheromones();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
