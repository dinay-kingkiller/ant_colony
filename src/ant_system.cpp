#include <cmath>
#include <vector>

#include "ros/ros.h"
#include "ant_colony/Pheromone.h"
#include "ant_colony/WhereNext.h"

// Algorithm Parameters
int VertexCount;
int EdgeCount;
int MaxEdgeWeight;
float DistancePower;
float EvaporationPower;
float PheromonePower;

// distances describes the simple, undirected, connected, weighted graph as an adjacency matrix.
// distances[i][j] is the time it takes to go from i to j.
// If the path is untraversable distances[i][j]==0.
// The graph is undirected: distances[i][j]==distances[j][i]
// The graph is simple (no loops): distances[i][i]==0
std::vector<std::vector<int>> distances;
std::vector<std::vector<float>> pheromones;
// desirability is a matrix of intermediate calculations:
//  desirability[i][j] = 1.0/distance[i][j]**DistancePower.
std::vector<std::vector<float>> desirability;

void GenerateMap() {
  uint8_t from_node;
  uint8_t to_node;
  distances.resize(VertexCount, std::vector<int>(VertexCount, 0));
  pheromones.resize(VertexCount, std::vector<float>(VertexCount, 0.0));
  desirability.resize(VertexCount, std::vector<float>(VertexCount, 0.0));
  
  // Generate a connected graph.
  for (int i = 1; i < VertexCount; ++i) {
    from_node = rand() % i;
    distances[from_node][i] = rand() % MaxEdgeWeight + 1;
    distances[i][from_node] = distances[from_node][i];
    pheromones[from_node][i] = 1.0;
    pheromones[i][from_node] = 1.0;
    desirability[from_node][i] = std::pow(distances[from_node][i], -DistancePower);
    desirability[i][from_node] = std::pow(distances[i][from_node], -DistancePower);
  }

  // Generate remaining edges.
  int remaining_locations;
  int location;
  for (int i = 0; i < EdgeCount-VertexCount; ++i) {
    remaining_locations = VertexCount*VertexCount - 2*VertexCount + 2 - 2*i;
    location = rand() % remaining_locations;
    for (int j = location; j == 0; --j) {
      if (j/VertexCount == j%VertexCount) ++location;
      if (distances[j/VertexCount][j%VertexCount]!=0) ++location;
    }
    from_node = location / VertexCount;
    to_node = location % VertexCount;
    distances[from_node][to_node] = rand() % MaxEdgeWeight + 1;
    distances[to_node][from_node] = distances[from_node][i];
    pheromones[from_node][to_node] = 1.0;
    pheromones[to_node][from_node] = 1.0;
    desirability[from_node][to_node] = std::pow(distances[from_node][to_node], -DistancePower);
    desirability[to_node][from_node] = std::pow(distances[to_node][from_node], -DistancePower);
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

bool ChoosePath(ant_colony::WhereNext::Request &req,
		 ant_colony::WhereNext::Response &res) {
  int sum=0;
  for (int i = 0; i < VertexCount; ++i) {
    sum += std::pow(pheromones[req.start_vertex][i], PheromonePower) * desirability[req.start_vertex][i];
  }
  int choice = rand() * sum;
  
  sum = 0;
  for (int i = 0; i < VertexCount; ++i) {
    sum += std::pow(pheromones[req.start_vertex][i], PheromonePower) * desirability[req.start_vertex][i];
    if (choice < sum) {
      res.next_vertex = i;
      res.edge_weight = distances[req.start_vertex][i];
      return true;
    }
  }
  return false;
}

int main(int argc, char **argv) {
  // Start up map node.
  ros::init(argc, argv, "map");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);
  
  // Set global parameters.
  nh.getParam("ant_colony/VertexCount", VertexCount);
  nh.getParam("ant_colony/EdgeCount", EdgeCount);
  nh.getParam("ant_colony/MaxEdgeWeight", MaxEdgeWeight);
  nh.getParam("ant_colony/DistancePower", DistancePower);
  nh.getParam("ant_colony/EvaporationPower", EvaporationPower);
  nh.getParam("ant_colony/PheromonePower", PheromonePower);

  GenerateMap();

  ros::Subscriber pheromone_drops = nh.subscribe("pheromone_deposits", 1000, AddPheromones);
  ros::ServiceServer where_next_server = nh.advertiseService("where_next", ChoosePath);
  
  while (ros::ok()) {
    UpdatePheromones();
    loop_rate.sleep();
  }
  return 0;
}
