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

/// \brief A traveling salesman ant.
///
/// ants should hold information about the problem to be solved, while the map
/// files hold information on the algorithm used to solve.


#include <random>
#include <set>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "ant_colony/Directions.h"
#include "ant_colony/Choices.h"
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

class traveler {
public:
  std::mt19937 mt_rand;
  int vertex_count;
  traveler() {
    std::random_device rd;
    mt_rand(rd());
  }
  int choose_path(ant_colony::Choices srv) {
    float sum_a;
    float sum_d;
    for (int i = 0; i < VertexCount; ++i) {
      
    }
    return mt_rand();
  }
};


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
  ros::Publisher smelly_ant = nh.advertise<ant_colony::PheromonePath>("path_pheromones", 1000);
  ros::Publisher loud_ant = nh.advertise<ant_colony::Location>("location", 1000);
  nh.getParam("VertexCount", VertexCount);
  nh.getParam("RewardPower", RewardPower);

  ROS_INFO("Waiting for directions service.");
  ros::service::waitForService("directions", 1000000);
  ROS_INFO("Directions service connected!");
  
  while (ros::ok()) {
    // ant.choose_path();
    // ant.travel_path();
    // ant.update_path();
    // if ant.is_at_goal() {
    // ant.reset_tour();
    // ant.update_map();
    // }
    // The ant wants to know where to go next.
    directions_srv.request.skip_here = tour;
    directions_srv.request.from_here = current_vertex;
    ROS_INFO("Waiting for directions...");
    if (not lost_ant.call(directions_srv)) {
      ROS_ERROR("Could not talk to Directions service.");
      return 1;
    }
    ROS_INFO("Received directions");
    next_vertex = directions_srv.response.go_here;
    travel_time = directions_srv.response.travel_time;

    // ant.travel();
    // The ant is traveling.
    location_msg.name = ros::this_node::getName();
    location_msg.from_vertex = current_vertex;
    location_msg.to_vertex = next_vertex;
    ROS_INFO_STREAM(location_msg.name<<" is Traveling.");
    ROS_INFO_STREAM("From "<<location_msg.from_vertex);
    ROS_INFO_STREAM("To "<<location_msg.to_vertex);
    loud_ant.publish(location_msg);
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
      smelly_ant.publish(pheromone_msg);

      // Reset trail
      tour_length = 0;
      tour.clear();
      tour.push_back(0);
      visited.clear();
      visited.insert(0);
    }    
  }
  return 0;
}
