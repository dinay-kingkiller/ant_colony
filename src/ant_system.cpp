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
/// 1. This node subscribes to Pheromone msgs to update the pheromone map.
/// 2. This node provides a direction service to ant nodes.
///
/// When this node is launched it will generate a randomized map for ant travel.
/// This map will track ant pheromones for the direction service. Every second,
/// the pheromones will decay according to the EvaporationPower parameter.

#include <cmath>
#include <ctime>
#include <vector>

#include "ros/ros.h"
#include "ant_colony/Pheromone.h"
#include "ant_colony/Directions.h"
#include "ant_colony/graph.h"

// Algorithm Parameters
int VertexCount;
int EdgeCount;
int MaxEdgeWeight;
int size_x;
int size_y;
float DistancePower;
float EvaporationPower;
float PheromonePower;

// distances is an adjacency matrix for finding edges.
std::vector<std::vector<int>> distances;
std::vector<std::vector<float>> pheromones;
// desirability is a matrix of intermediate calculations:
//  desirability[i][j] = 1.0/distance[i][j]**DistancePower.
std::vector<std::vector<float>> desirability;

void SetupMap() {
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

void AddPheromones(const ant_colony::Pheromone::ConstPtr& msg) {
  float max;
  int i;
  int j;
  pheromone_i = msg->from_vertex * VertexCount + msg->to_vertex;
  pheromone_j = msg->to_vertex * VertexCount + msg->from_vertex;
  pheromones[pheromone_i] += msg->deposit;
  pheromones[pheromone_j] += msg->deposit;
  max = pheromones[pheromone_i];
  // TODO: Verify pheromones don't go up if not deposited.
  for (int i = 0; i < VertexCount**VertexCount; ++i) {
    if (pheromeones[i] < .001) {
      pheromones[i] = 0.0;
    }
    else if (pheromones[i] + .001 > max) {
      ROS_WARN_STREAM("Possible max'd value"<<pheromones[i]/max);
      ROS_WARN_STREAM("i: "<<i/VertexCount<<" j: "<<j%VertexCount);
      pheromones[i] = 1.0;
    }
    else {
      pheromones[i] = pheromones[i] / max;
    }
  }
}

void GetPlottable(vector<int> plot_data) {
  
  
}

void UpdatePheromones() {
  for (int i = 0; i < VertexCount; ++i) {
    for (int j = i; j < VertexCount; ++j) {
      pheromones[i][j] = (1-EvaporationPower) * pheromones[i][j];
    }
  }
}

bool ChoosePath(ant_colony::Directions::Request &req,
		 ant_colony::Directions::Response &res) {
  int start = req.from_here;
  double attraction;
  double attraction_ttl = 0.0;
  double desirability_ttl = 0.0;
  double sum;
  double choice = rand() * 1.0 / RAND_MAX;

  for (int i = 0; i < VertexCount; ++i) {
    attraction = std::pow(pheromones[start][i], PheromonePower) * desirability[start][i];
    attraction_ttl += attraction;
    desirability_ttl += desirability[start][i];
  }
  
  ROS_INFO_STREAM("Choice: "<<choice);
  sum = 0.0;
  for (int i = 0; i < VertexCount; ++i) {
    attraction = std::pow(pheromones[start][i], PheromonePower) * desirability[start][i];
    ROS_INFO_STREAM("Desirability: "<<desirability[start][i]);
    ROS_INFO_STREAM("Attraction: "<<attraction);
    if (attraction / attraction_ttl < 0.001) {
      continue;
    }
    sum += attraction;
    if (sum / attraction_ttl + 0.001 > choice) {
      res.go_here = i;
      res.travel_time = distances[start][i];
      ROS_INFO_STREAM("From Here: "<<req.from_here);
      ROS_INFO_STREAM("Go Here: "<<res.go_here);
      return true;
    }
  }
  
  ROS_INFO_STREAM("sum: "<<sum);
  ROS_WARN("Not enough pheromones. Using desirability.");
  sum = 0.0;
  for (int i = 0; i < VertexCount; ++i) {
    if (desirability[start][i] / desirability_ttl < 0.000001) {
      continue;
    }
    sum += desirability[start][i];
    if (sum / desirability_ttl > choice) {
      res.go_here = i;
      res.travel_time = distances[start][i];
      ROS_INFO_STREAM("From Here: "<<req.from_here);
      ROS_INFO_STREAM("Go Here: "<<res.go_here);
      return true;
    }
  }
  ROS_INFO_STREAM("sum: "<<sum);
  ROS_ERROR("Could not find best path.");
  return false;
}

int main(int argc, char **argv) {
  // Start up map node.
  ros::init(argc, argv, "map");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);
  srand(time(NULL));

  // Set global parameters.
  nh.getParam("/VertexCount", VertexCount);
  nh.getParam("/EdgeCount", EdgeCount);
  nh.getParam("/DistancePower", DistancePower);
  nh.getParam("/EvaporationPower", EvaporationPower);
  nh.getParam("/PheromonePower", PheromonePower);

  // Setup Graph
  std::string GraphType;
  nh.getParam("/GraphType", GraphType);
  if (GraphType == "Incomplete") {
    nh.getParam("/MaxEdgeWeight", MaxEdgeWeight);
    GenerateRandomGraph(distances, VertexCount, MaxEdgeWeight);
  }
  else if (GraphType == "Flat") {
    nh.getParam("ant_colony/size_x", size_x);
    nh.getParam("ant_colony/size_y", size_y);
  }  
  SetupMap();

  ros::Subscriber pheromone_drops = nh.subscribe("pheromones", 1000, AddPheromones);
  ros::ServiceServer scent_trail = nh.advertiseService("directions", ChoosePath);
  ros::spin();
  while (ros::ok()) {
    UpdatePheromones();


    
    loop_rate.sleep();
  }
  return 0;
}
