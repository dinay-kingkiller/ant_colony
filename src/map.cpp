#include "ros/ros.h"
#include "ant_colony/pheromone.h"
#include "ant_colony/where_next.h"

// distances describes the simple, undirected, connected, weighted graph as an adjacency matrix.
// distances[i][j] is the time it takes to go from i to j.
// If the path is untraversable distances[i][j]==0.
// The graph is undirected: distances[i][j]==distances[j][i]
// The graph is simple (no loops): distances[i][i]==0
uint8_t distances[VERTEX_COUNT][VERTEX_COUNT];
// pheromones 

float pheromones[VERTEX_COUNT][VERTEX_COUNT];
float desirability[VERTEX_COUNT][VERTEX_COUNT];

void GenerateMap() {
  uint8_t from_node;
  uint8_t to_node;
  // Initialize matrices
  for (int i = 0; i < VERTEX_COUNT; ++i) {
    for (int j = 0; j < VERTEX_COUNT; ++j) {
      paths[i][j] = 0;
      pheromones[i][j] = 0.0;
      desirability[i][j] = 0.0;
    }
  }
  
  // Generate a connected graph.
  for (int i = 1; i < VERTEX_COUNT; ++i) {
    from_node = rand() % i;
    distances[from_node][i] = rand() % MAX_WEIGHT + 1;
    distances[i][from_node] = distances[from_node][i];
    pheromones[from_node][i] = 1.0;
    pheromones[i][from_node] = 1.0;
    desirability[from_node][i] = 1.0 / distance[from_node]**DISTANCE_POWER;
    desirability[i][from_node] = 1.0 / distances[i][from_node]**DISTANCE_POWER;
  }

  // Generate remaining edges.
  for (int i = 0; i < EDGE_COUNT-VERTEX_COUNT; ++i) {
    remaining_locations = VERTEX_COUNT**2 - 2*VERTEX_COUNT + 2 - 2*i;
    location = rand() % remaining_locations;
    for (int j = location; j == 0; --j) {
      if (j/VERTEX_COUNT == j%VERTEX_COUNT) ++location;
      if (distances[j/VERTEX_COUNT][j%VERTEX_COUNT]!=0) ++location;
    }
    from_node = j / VERTEX_COUNT;
    to_node = j % VERTEX_COUNT;
    distances[from_node][to_node] = rand() % MAX_WEIGHT + 1;
    distances[to_node][from_node] = distances[from_node][i];
    pheromones[from_node][to_node] = 1.0;
    pheromones[to_node][from_node] = 1.0;
    desirability[from_node][to_node] = 1.0 / distances[from_node][to_node]**DISTANCE_POWER;
    desirability[to_node][from_node] = 1.0 / distances[to_node][from_node]**DISTANCE_POWER;
  }
}

void AddPheromone(const ant_colony::pheromone::ConstPtr& msg) {
  msg.frome_node;
  msg.to_node;
  msg.deposit;
}

void ChoosePath

int main(int argc, char **argv) {
  // Generate map.

  ros::init(argc, argv, "map");
  ros::NodeHandle nh;
  ros::Subscriber pheromone_updates = nh.subscribe("pheromone_drops", BUFFER_SIZE, AddPheromone);

}
