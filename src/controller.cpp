using namespace std;
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <unistd.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <second_assignment/service.h>
#include <stdlib.h>
#include <std_srvs/Empty.h>
#include <ncurses.h>

ros::Publisher publisher; // Publisher
geometry_msgs::Twist vel_msg; // Message to sent

float front_dist; // Declaring variables for distance calculation from 3 sides
float left_dist;
float right_dist;

float border_th = 0.7;
float old_speed = 0.0; // Old speed
float new_speed = 0.0; // New speed

bool speed_status(second_assignment::service::Request &request, second_assignment::service::Response &serv_res) { // Speed control and update
      new_speed = old_speed + request.reg; // Summarize our old speed + 0.25 each time after pressin UP key to obtain new speed
      serv_res.cont = 1;
      if (new_speed < old_speed) {
        if (new_speed <= 0) { // If new speed is less or equal to 0, then stop robot to prevent it from moving in opposite direction
          new_speed = 0.0;
        }
        else { // If new speed is more than 0, but less than old speed, then indicate that speed has been decreased
          ROS_INFO("Status: Speed has been decreased");
        }
      }
      else if (new_speed > old_speed) { // If new speed is higher than old one, then indicate that speed has been increased
        ROS_INFO("Status: Speed has been increased");
      }
      old_speed = new_speed; // Updates speed value
      return true;
}

vector<float> dist_approximation(vector<float> dist_info) { // Calculates distances from left, right, and front sides
    vector<float> dist_vtr = {};
    front_dist = 40.0, left_dist = 40.0, right_dist = 40.0;
        
    for (int i = 0; i < 180.0; i = i + 1) { // Takes approximation between 0° and 45° for the left side
      if (left_dist > dist_info[i]) {
        left_dist = dist_info[i]; // Distance to the left side
      }
    }
    dist_vtr.insert(dist_vtr.begin(), left_dist); // Inserts it to the vector as a 1st element 
    
    for (int i = 180.0; i < 540.0; i = i + 1) { // Takes approximation between 45° and 135° for the front side
      if (front_dist > dist_info[i]) {
        front_dist = dist_info[i]; // Distance to the front side
      }
    }
    dist_vtr.insert(dist_vtr.begin() + 1, front_dist); // Inserts it to the vector as a 2nd element 
    
    for (int i = 540.0; i < 721.0; i = i + 1) { // Takes approximation between 135° and 180° for the right side
      if (right_dist > dist_info[i]) {
        right_dist = dist_info[i]; // Distance to the right side
      }
    }
    dist_vtr.insert(dist_vtr.begin() + 2, right_dist); // Inserts it to the vector as a 3rd element 
    return dist_vtr; // Returns vector float with 3 values for left, front, right sides
}

void decision_maker(const sensor_msgs::LaserScan::ConstPtr& dist_info) { // Robot rotation controller
  vector<float> max_nearest_dist = dist_approximation(dist_info->ranges); // Calculates distance from dist_info 3 sides view (front, left, right)
  
  if (max_nearest_dist[1] > border_th) { // If threshold value in front isn't reached, then moves straight without rotation
    vel_msg.angular.z = 0;
    vel_msg.linear.x = new_speed;
  }
  else {
    if (max_nearest_dist[0] != max_nearest_dist[2]) {
      if (max_nearest_dist[0] < max_nearest_dist[2]) { // If close to border from the right side, then turn left 
        ROS_INFO("Turn left");
        vel_msg.angular.z = 3;
        vel_msg.linear.x = 0.25;
      }
      else { // If close to border from the left side, then turn right
        ROS_INFO("Turn right");
        vel_msg.angular.z = -3;
        vel_msg.linear.x = 0.25;
      }
    }
  }
  publisher.publish(vel_msg); 
}

int main(int argc, char **argv) {
  system("clear");
  ros::init(argc, argv, "Controller");
  ros::NodeHandle node_handle;
  publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // Publishes to /cmd_vel at rate 1
  ros::Subscriber subscriber = node_handle.subscribe("/base_scan", 1, decision_maker); 
  ros::ServiceServer service = node_handle.advertiseService("/robot_speed", speed_status); 
  ros::spin();
  return 0;
}