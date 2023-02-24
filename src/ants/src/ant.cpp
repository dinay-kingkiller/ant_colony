// TODO MOVE TO CONFIG
#define BUFFER_SIZE 100000
#define VERTEX_COUNT 256

#include <string>
#include <vector>

#include "ros/ros.h"
#include "ant_colony/pheromone.h"
#include "ant_colony/where_next.h"


int main(int argc, char **argv) {
  // Setup this ant
  std::string name = argv[1];
  ros::init(argc, argv, name);

  // Setup communication with ROS
  ros::NodeHandle nh;
  ros::Publisher reporter = nh.advertise<ant_colony::pheromone>("paths_checked", BUFFER_SIZE);
  ros::ServiceClient lost_ant = nh.serviceClient<ant_colony::where_next>("where_next");
  ant_colony::where_next where_next_srv;
  
  bool exploring = true;
  int start_vertex = 0;
  int current_vertex = start_vertex;
  int goal_vertex = VERTEX_COUNT;
  std::vector<int> tour;
  std::vector<int> path_length;
  
  while (ros::ok()) {
    if (exploring) {
      // The ant wants to know where to go next.
      where_next_srv.start_vertex = current_vertex;
      if (not lost_ant.call(where_next_srv)) {
	ROS_ERROR(name+" could not talk to where_next service.");
	return 1;
      }
      ros::Duration(where_next_srv.response.travel_time).sleep();
      current_vertex = where_next_srv.response.next_vertex;
      if (current_vertex == goal_vertex) {
	exploring = false;
      }
    }
    else {
      // The ant is returning, spreading the good news!
      if (current_vertex == start_vertex) {
	exploring = true;
      }
    }
    
  }
  return 0;
}
