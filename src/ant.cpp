// TODO MOVE TO CONFIG
#define BUFFER_SIZE 100000
#define VERTEX_COUNT 256
#define REWARD_POWER 1

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
  ros::Publisher reporter = nh.advertise<ant_colony::pheromone>("pheromone_drops", BUFFER_SIZE);
  ant_colony::pheromone pheromone_msg;
  ros::ServiceClient lost_ant = nh.serviceClient<ant_colony::where_next>("where_next");
  ant_colony::where_next where_next_srv;
  
  bool exploring = true;
  int start_vertex = 0;
  int travel_time;
  int current_vertex = start_vertex;
  int next_vertex;
  int goal_vertex = VERTEX_COUNT;
  std::vector<int> tour = {};
  std::vector<int> path_length = {};
  int tour_length = 0;
  
  while (ros::ok()) {
    if (exploring) {
      // The ant wants to know where to go next.
      where_next_srv.start_vertex = current_vertex;
      if (not lost_ant.call(where_next_srv)) {
	ROS_ERROR(name + " could not talk to where_next service.");
	return 1;
      }
      travel_time = where_next_srv.response.travel_time;
      // The ant is traveling.
      ros::Duration(travel_time).sleep();
      next_vertex = where_next_srv.response.next_vertex;
      path_length.push_back(travel_time);
      tour.push_back(next_vertex);
      tour_length += path_time;
      current_vertex = next_vertex;
      if (current_vertex == goal_vertex) {
	exploring = false;
      }
    }
    else {
      // The ant is spreading the good news!
      travel_time = path_length.pop_back();
      ros::Duration(travel_time).sleep();
      next_vertex = tour.pop_back();
      pheromone_msg.start_vertex = current_vertex;
      pheromone_msg.next_vertex = next_vertex;
      pheromone_msg.drop = REWARD_POWER / tour_length;
      if (current_vertex == start_vertex) {
	exploring = true;
	tour.clear();
	path_length.clear();
	tour_length = 0;
      }
    }
    
  }
  return 0;
}
