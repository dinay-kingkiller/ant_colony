#include <string>
#include <vector>

#include "ros/ros.h"
#include "ant_colony/Pheromone.h"
#include "ant_colony/WhereNext.h"


int main(int argc, char **argv) {
  int VertexCount;
  float RewardPower;
  
  // Setup this ant
  std::string name = argv[1];
  ros::init(argc, argv, name);

  // Setup communication with ROS
  ros::NodeHandle nh;
  nh.getParam("ant_colony/VertexCount", VertexCount);
  nh.getParam("ant_colony/RewardPower", RewardPower);
  ros::Publisher reporter = nh.advertise<ant_colony::Pheromone>("pheromone_deposits", 1000);
  ant_colony::Pheromone pheromone_msg;
  ros::ServiceClient lost_ant = nh.serviceClient<ant_colony::WhereNext>("where_next");
  ant_colony::WhereNext where_next_srv;
  
  bool exploring = true;
  int start_vertex = 0;
  int travel_time;
  int current_vertex = start_vertex;
  int next_vertex;
  int goal_vertex = VertexCount;
  std::vector<int> tour = {};
  std::vector<int> path_length = {};
  int tour_length = 0;
  
  while (ros::ok()) {
    if (exploring) {
      // The ant wants to know where to go next.
      where_next_srv.request.start_vertex = current_vertex;
      if (not lost_ant.call(where_next_srv)) {
	// ROS_ERROR(name + " could not talk to where_next service.");
	return 1;
      }
      travel_time = where_next_srv.response.edge_weight;
      // The ant is traveling.
      ros::Duration(travel_time).sleep();
      next_vertex = where_next_srv.response.next_vertex;
      path_length.push_back(travel_time);
      tour.push_back(next_vertex);
      tour_length += travel_time;
      current_vertex = next_vertex;
      if (current_vertex == goal_vertex) {
	exploring = false;
      }
    }
    else {
      // The ant is spreading the good news!
      travel_time = path_length.back();
      path_length.pop_back();
      ros::Duration(travel_time).sleep();
      next_vertex = tour.back();
      tour.pop_back();
      pheromone_msg.from_vertex = current_vertex;
      pheromone_msg.to_vertex = next_vertex;
      pheromone_msg.deposit = RewardPower / tour_length;
      reporter.publish(pheromone_msg);
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
