#include <cmath>
#include <ctime>
#include <map>
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
  pheromones[msg->from_vertex][msg->to_vertex] += msg->deposit;
  pheromones[msg->to_vertex][msg->from_vertex] += msg->deposit;
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
    if (attraction / attraction_ttl < 0.0001) {
      continue;
    }
    sum += attraction;
    if (sum / attraction_ttl + 0.0001 > choice) {
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
