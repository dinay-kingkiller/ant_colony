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

#include <set>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "ant_colony/Directions.h"
#include "ant_colony/Location.h"
#include "ant_colony/PheromonePath.h"

int current_vertex = 0; // Start at the beginning
int next_vertex;
float travel_time;
std::set<int> visited = {0}; // Visited vertices
std::vector<int> tour = {0};
float tour_length = 0;
int VertexCount;
float RewardPower;

int main(int argc, char **argv) {
  // Start talking with ROS.
  ros::init(argc, argv, "Princess"); 
  ros::NodeHandle nh;
  ros::Rate ant_speed(1);

  // ROS Communication
  ant_colony::PheromonePath pheromone_msg;
  ant_colony::Location location_msg;
  ant_colony::Directions directions_srv;
  ros::ServiceClient lost_ant = nh.serviceClient<ant_colony::Directions>("directions");
  ros::Publisher pheromone_pub = nh.advertise<ant_colony::PheromonePath>("path_pheromones", 1000);
  ros::Publisher location_pub = nh.advertise<ant_colony::Location>("location", 1000);
  nh.getParam("VertexCount", VertexCount);
  nh.getParam("RewardPower", RewardPower);

  ros::service::waitForService("directions", 1000000);
  bool exploring = true;
  while (ros::ok()) {
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
    ROS_DEBUG_STREAM(location_msg.name<<" is Traveling.");
    ROS_DEBUG_STREAM("From "<<location_msg.from_vertex);
    ROS_DEBUG_STREAM("To "<<location_msg.to_vertex);
    location_pub.publish(location_msg);
    ros::Duration(travel_time).sleep();

    // Update the ant tour.
    tour.push_back(next_vertex);
    visited.insert(next_vertex);
    tour_length += travel_time;
    current_vertex = next_vertex;
    
    if (visited.size() == VertexCount && current_vertex==0) {
      // The ant marks it trail.
      pheromone_msg.tour = tour;
      pheromone_msg.deposit = RewardPower / tour_length; 
    }
    
  }
  return 0;
}
