#include "ros/node_handle.h"
#include <robot_gui/robot_gui.h>
#include <thread>

using namespace std;

int main(int argc, char **argv) {
  // make a gui object
  RobotGUI ui;

  // init ROS node
  ros::init(argc, argv, "UI_main_node");

  // init node handle
  ros::NodeHandle info_n("UI_info_sub_node");
  ros::NodeHandle vel_n("UI_vel_pub_node");
  ros::NodeHandle odom_n("UI_odom_sub_node");
  ros::NodeHandle dis_n("UI_dis_srv_node");

  string info_topic = "/robot_info";
  string vel_topic = "/cmd_vel";
  string odom_topic = "/odom";
  string distance_topic = "/get_distance";

  // init info

  ui.initInfoSub(info_n, info_topic);
  ui.initVelControl(vel_n, vel_topic);
  ui.initOdomSub(odom_n, odom_topic);
  ui.initDistanceClient(dis_n, distance_topic);
  ui.DisplayUI();
  return 0;
}