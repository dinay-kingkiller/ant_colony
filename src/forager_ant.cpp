// BSD 3-Clause License
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
int VertexCount; 
float RewardPower;


// Ant on Tour
std::vector<int> tour = {0};
std::vector<int> path_length = {0};
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
  nh.getParam("VertexCount", VertexCount);
  nh.getParam("RewardPower", RewardPower);
  goal_vertex = VertexCount - 1;

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
      for (int i = 0; i < travel_time; ++i) {
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
	location_msg.name = ros::this_node::getName();
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
      if (tour.size() == 0) {
	exploring = true;
	tour.clear();
	path_length.clear();
	tour_length = 0;
      }
    }
    
  }
  return 0;
}
