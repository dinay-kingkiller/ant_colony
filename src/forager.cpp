#include <string>
#include <vector>

#include "ros/ros.h"
#include "ant_colony/Directions.h"
#include "ant_colony/Location.h"
#include "ant_colony/Pheromone.h"

// ROS Communication
ros::NodeHandle nh;
ros::Publisher pheromone_pub;
ros::Publisher location_pub;
ros::ServiceClient lost_ant;
ros::Rate ant_speed(1);
ant_colony::Pheromone pheromone_msg;
ant_colony::Location location_msg;
ant_colony::Directions directions_srv;

// Important Places
int start_vertex = 0;
int goal_vertex;
int current_vertex = start_vertex;
int next_vertex;

// Ant on Tour
std::vector<int> tour = {};
std::vector<int> path_length = {};
int tour_length = 0;

//
float RewardPower;



int main(int argc, char **argv) {
  // Start talking with ROS.
  ros::init(argc, argv, "Princess");
  lost_ant = nh.serviceClient<ant_colony::Directions>("directions");
  reporter = nh.advertise<ant_colony::Pheromone>("pheromone_deposits", 1000);
  nh.getParam("ant_colony/VertexCount", goal_vertex);
  nh.getParam("ant_colony/RewardPower", RewardPower);
  
  
  while (ros::ok()) {
    if (exploring) {
      // The ant wants to know where to go next.
      directions_srv.request.from_here = current_vertex;
      if (not lost_ant.call(directions_srv)) {
	// ROS_ERROR(name + " could not talk to where_next service.");
	return 1;
      }
      next_vertex = where_next_srv.response.go_here;
      travel_time = where_next_srv.response.travel_time;
      // The ant is traveling.
      for (int i = 0; i < travel_time; ++i) {
	location_msg.from_vertex = current_vertex;
	location_msg.to_vertex = next_vertex;
	location_msg.progress = i;
	location_pub.publish(location_msg);
	ant_speed.sleep();
      }
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
      next_vertex = tour.back();
      tour.pop_back();
      // The ant is traveling.
      for (int i = 0; i < travel_time; ++i) {
	location_msg.from_vertex = current_vertex;
	location_msg.to_vertex = next_vertex;
	location_msg.progress = i;
	location_pub.publish(location_msg);
	ant_speed.sleep();
      }
      pheromone_msg.from_vertex = current_vertex;
      pheromone_msg.to_vertex = next_vertex;
      pheromone_msg.deposit = RewardPower / tour_length;
      pheromone_pub.publish(pheromone_msg);
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
