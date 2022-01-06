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

float robot_speed = 0.0;
std_srvs::Empty reset;

void instructions() {
  system("clear");
  printw("You can press the following buttons in order to control the robot: \n");
  printw("Press UP arrow key for speed increase \n");
  printw("Press DOWN arrow key for speed decrease \n");
  printw("Press R button for placing a robot to initial position \n");
  printw("Press Q button to exit from the terminal \n");
}

int main(int argc, char **argv) {
  // Initialization for arrow keys 
  int ch; // Character that will be laterly used as control of Robot in'while' loop
  initscr(); // Initialization
  raw(); // Ignores when user presses 'Ctrl + C' command to exit from the programm
  keypad(stdscr, TRUE); // Gives reading of KEY buttons
  noecho();
  instructions();
  
  ros::init(argc, argv, "Regulator"); // ROS initialization function
  ros::NodeHandle node_handle; // Starts the node (As well as shuts it down)
  ros::ServiceClient service_client = node_handle.serviceClient<second_assignment::service>("/robot_speed"); // The service is created and advertised over ROS
  second_assignment::service service; 
  service_client.waitForExistence(); // Wait for this service to be advertised and available
  int isExit = 0; // Introduced in order to terminate program after pressing Q button
  
  while ((ros::ok()) && ((ch = getch()) != '#')) {
    switch(ch) {
      case KEY_UP: // UP arrow key
        robot_speed = 0.25;
        ROS_INFO("Speed has been increased \r\n");
        break;
      case KEY_DOWN: // Down arrow key
        robot_speed = -0.25;
        ROS_INFO("Speed has been decreased \r\n");
        break;
      case 82: case 114: // R or r button
        robot_speed = 0.0;
        ros::service::call("/reset_positions", reset);
        ROS_INFO("Robot returned to initial position \r\n");
        break;
      case 81: case 113: // Q or q button
        ROS_INFO("The program is going to close \r\n");
        isExit = 1;
        break;
      default: // Otherwise
        ROS_INFO("Unrecognized command, please press the correct button! \r\n");
        break;
    }
    if (isExit) { // If Q is pressed then breaks the loop and exits
      break;
    }
    if (robot_speed < 0 || robot_speed > 0) {
      service.request.reg = robot_speed;
      if (service_client.call(service)) { 
        // Sent speed value to controller via srv successfully
      } 
      else {
        ROS_INFO("Warning! Error occured during communication. \r\n");
      }
    }
  }
  refresh(); // Print it on to the real screen 
  getch(); // Waits for the user's input
  endwin(); // End curses mode
  return 0;
}