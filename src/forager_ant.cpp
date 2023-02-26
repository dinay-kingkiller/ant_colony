#include <string>
#include <vector>

#include "ros/ros.h"
#include "ant_colony/Directions.h"
#include "ant_colony/Location.h"
#include "ant_colony/Pheromone.h"


// Important Places
int start_vertex = 0;
int goal_vertex;
int current_vertex = start_vertex;
int next_vertex;
int travel_time;
float RewardPower;


// Ant on Tour
std::vector<int> tour = {};
std::vector<int> path_length = {};
int tour_length = 0;

int main(int argc, char **argv) {
  // Start talking with ROS.
  ros::init(argc, argv, "Princess"); 
  ros::NodeHandle nh;
  ros::Rate ant_speed(1);
  // ROS Communication
  ant_colony::Pheromone pheromone_msg;
  ant_colony::Location location_msg;
  ant_colony::Directions directions_srv;
  ros::ServiceClient lost_ant = nh.serviceClient<ant_colony::Directions>("directions");
  ros::Publisher pheromone_pub = nh.advertise<ant_colony::Pheromone>("pheromones", 1000);
  ros::Publisher location_pub = nh.advertise<ant_colony::Location>("location", 1000);
  nh.getParam("ant_colony/VertexCount", goal_vertex);
  nh.getParam("ant_colony/RewardPower", RewardPower);

  ros::service::waitForService("directions", 1000000);
  bool exploring = true;
  while (ros::ok()) {
    if (exploring) {
      // The ant wants to know where to go next.
      directions_srv.request.from_here = current_vertex;
      if (not lost_ant.call(directions_srv)) {
	ROS_ERROR("Could not talk to Directions service.");
	return 1;
      }
      next_vertex = directions_srv.response.go_here;
      travel_time = directions_srv.response.travel_time;
      std::string test = std::to_string(next_vertex);

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
