// Generate a new ant publisher.
#include <string>

#include "ros/ros.h"
#include "ants/pheromone.h"
#include "ants/explore.h"


int main(int argc, char **argv) {
  std::string name = argv[1];
  ros::init(argc, argv, name);
  ros::NodeHandle ant;
  ros::Publisher report = ant.advertise<ants::pheromone>("checked", BUFFER_SIZE);
  ros::Rate travel_rate(1); // Ants travel one unit per second.
  while (ros::ok()) {
    if (exploring) {
      if (next_node == goal_node) {
	}
    }
}
