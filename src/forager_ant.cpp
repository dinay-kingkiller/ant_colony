#include <string>
#include <vector>

#include "ros/ros.h"
#include "ant_colony/Directions.h"
#include "ant_colony/Location.h"
#include "ant_colony/Pheromone.h"


// Important Places
uint16_t start_vertex = 0;
uint16_t goal_vertex;
uint16_t current_vertex = start_vertex;
uint16_t next_vertex;
uint16_t travel_time;
int VertexCount; 
float RewardPower;


// Ant on Tour
std::vector<uint16_t> tour = {};
std::vector<uint16_t> path_length = {};
uint16_t tour_length = 0;

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
  nh.getParam("VertexCount", VertexCount);
  nh.getParam("RewardPower", RewardPower);
  goal_vertex = VertexCount - 1;
  ROS_INFO_STREAM("VC: "<<VertexCount);
  ROS_INFO_STREAM("GV: "<<goal_vertex);


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
      
      // The ant is traveling.
      location_msg.name = ros::this_node::getName();
      location_msg.from_vertex = current_vertex;
      location_msg.to_vertex = next_vertex;
      for (uint16_t i = 0; i < travel_time; ++i) {
	location_msg.progress = i;
	location_pub.publish(location_msg);
	ROS_INFO_STREAM("Exploring");
	ROS_INFO_STREAM("From Vertex: "<<location_msg.from_vertex);
	ROS_INFO_STREAM("To Vertex: "<<location_msg.to_vertex);
	ROS_INFO_STREAM("Progress: "<<location_msg.progress);
	ant_speed.sleep();
      }
      path_length.push_back(travel_time);
      tour.push_back(next_vertex);
      tour_length += travel_time;
      current_vertex = next_vertex;
      if (current_vertex == goal_vertex) {
	ROS_INFO_STREAM("GOAL!!! "<<goal_vertex);
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
      for (uint16_t i = 0; i < travel_time; ++i) {
	location_msg.name = ros::this_node::getName();
	location_msg.from_vertex = current_vertex;
	location_msg.to_vertex = next_vertex;
	location_msg.progress = i;
	location_pub.publish(location_msg);
	ROS_INFO_STREAM("Returning");
	ROS_INFO_STREAM("From Vertex: "<<location_msg.from_vertex);
	ROS_INFO_STREAM("To Vertex: "<<location_msg.to_vertex);
	ROS_INFO_STREAM("Progress: "<<location_msg.progress);
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
