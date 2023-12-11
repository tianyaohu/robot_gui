#include <iostream>
#include <ros/ros.h>
#include <sstream>

// include custome message
#include "robot_info/TwoStrVec.h"
#include "ros/node_handle.h"
#include "ros/service_client.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#define WINDOW_NAME "Frank's Robot GUI"

using namespace std;

class RobotGUI {
public:
  // constructor
  RobotGUI()
      : vec_info({"Info1: Not inited", "Info2: Not inited", "Info3: Not inited",
                  "Info4: Not inited"}),
        x(-1), y(-1), z(-1) {
    cout << "RobotGUI Constructor is called" << endl;
  };
  // destructor
  ~RobotGUI() { cout << "RobotGUI Destructor is called" << endl; };

  void initInfoSub(ros::NodeHandle &node, const string info_topic) {
    cout << "within info sub" << endl;
    this->info_topic = info_topic;
    // init info sub
    this->info_sub_ =
        node.subscribe(this->info_topic, 1, &RobotGUI::InfoCallback, this);
  };

  // can be templated to generalized to other publishers
  void initVelControl(ros::NodeHandle &node, const string topic2pub) {
    // advertize
    this->vel_pub_ = node.advertise<geometry_msgs::Twist>(topic2pub, 10);
  }

  // init odom sub
  void initOdomSub(ros::NodeHandle &node, const string odom_topic) {
    cout << "Started Odom Sub" << endl;
    this->odom_sub_ =
        node.subscribe(odom_topic, 1, &RobotGUI::OdomCallback, this);
  }

  void initDistanceClient(ros::NodeHandle &node, const string distance_topic) {
    // init distance client
    this->dis_client_ = node.serviceClient<std_srvs::Trigger>(distance_topic);
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // update position info
    this->x = msg->pose.pose.position.x;
    this->y = msg->pose.pose.position.y;
    this->z = msg->pose.pose.position.z;
  }

  void DisplayUI() {
    cout << "DisplayUI called" << endl;
    // init a padding paramter
    int y_padding = 15;
    int x_padding = 5;

    // init frame and window
    init_UI_frame(800, 400);
    cvui::init(WINDOW_NAME);

    // while esc is not pressed
    while (cv::waitKey(20) != 27) {
      // reset pos
      this->pos = cv::Point(x_padding, y_padding);
      // Fill the frame with a nice color
      frame = cv::Scalar(49, 52, 49);

      // info ui
      this->InfoUI(y_padding);

      // add y_padding
      this->pos.y += y_padding;

      // control section
      this->ControlUI(y_padding);

      // add y_padding
      this->pos.y += y_padding;

      // Velocity section
      this->VelUI(y_padding);

      // add y_padding
      this->pos.y += y_padding;

      // Odom Section
      this->OdomUI(y_padding);

      // add y_padding
      this->pos.y += y_padding;

      // Distance Button
      this->DistanceUI(y_padding);

      // update ui and show window
      cvui::update();
      cv::imshow(WINDOW_NAME, this->frame);

      // publish Twist
      vel_pub_.publish(msg_vel);

      ros::spinOnce();
    }
    cout << "DisplayUI termminated" << endl;
  }

private:
  ros::NodeHandle *m_ros_node_object;

  ros::Subscriber info_sub_;
  ros::Publisher vel_pub_;
  ros::Subscriber odom_sub_;
  ros::ServiceClient dis_client_;

  // UI contents:
  // string info topic
  string info_topic;
  // - general info
  vector<string> vec_info;

  // Twist msg
  geometry_msgs::Twist msg_vel;

  //  - Robot Position
  float x;
  float y;
  float z;
  // distance response
  string response;

  // UI frame
  int ui_height;
  int ui_width;
  cv::Mat frame;
  // init point for ui alignment
  cv::Point pos;

  void InfoCallback(const robot_info::TwoStrVec::ConstPtr &msg) {
    // storing the msg vec with m
    this->StoreMsgVec(msg->vec1, msg->vec2);
  }

  // private functions
  void StoreMsgVec(const vector<string> &header_vec,
                   const vector<string> &value_vec) {
    // clear the old info
    this->vec_info.clear();

    for (int i = 0; i < header_vec.size(); i++) {
      string temp = header_vec[i] + ":   " + value_vec[i];
      this->vec_info.push_back(temp);
    }
  };

  // init UI frame
  void init_UI_frame(int ui_height, int ui_width) {
    this->ui_height = ui_height;
    this->ui_width = ui_width;
    this->frame = cv::Mat(this->ui_height, this->ui_width, CV_8UC3);
  }

  // General Info
  void InfoUI(int width_padding) {
    // init needed x,y offset
    int x_offset = 5, line_space = 5, line_height = 15 + line_space;

    int window_width = ui_width - width_padding * 2;
    int window_height = (this->vec_info.size() + 1) * line_height +
                        5; // 5 is for the first half of line space
    //  the following should house info_topic string
    cvui::window(this->frame, this->pos.x, this->pos.y, window_width,
                 window_height, "info_topic_name");
    // adding half of line space
    this->pos.y += 5;
    // write all info lines;
    for (const std::string &row : vec_info) {
      this->pos.y += line_height; // 15 is standard text height
      cvui::text(this->frame, this->pos.x + x_offset, this->pos.y, row);
    }
    this->pos.y += line_height;
  }

  void ControlUI(int width_padding) {
    // init button width and height
    int button_width = this->ui_width / 3 - width_padding;
    int button_height = this->ui_height / (3 * 6);
    // vel step
    float vel_step = 0.2;

    // forward
    if (cvui::button(frame, this->pos.x + button_width + width_padding,
                     this->pos.y, button_width, button_height, "Forward")) {
      cout << "forward is pressed" << endl;
      this->msg_vel.linear.x += vel_step;
      cout << msg_vel.linear.x << endl;
    }
    // increment y
    this->pos.y += button_height + width_padding;
    // left + Stop + Right
    if (cvui::button(frame, this->pos.x, this->pos.y, button_width,
                     button_height, "Left")) {
      cout << "Left is pressed" << endl;
      this->msg_vel.angular.z += vel_step;
      cout << msg_vel.angular.z << endl;
    }

    if (cvui::button(frame, this->pos.x + button_width + width_padding,
                     this->pos.y, button_width, button_height, "Stop")) {
      cout << "Stop is pressed" << endl;
      this->msg_vel.angular.z = 0;
      this->msg_vel.linear.x = 0;
      cout << msg_vel.linear.x << endl;

      cout << msg_vel.angular.z << endl;
    }

    if (cvui::button(frame, this->pos.x + 2 * (button_width + width_padding),
                     this->pos.y, button_width, button_height, "Right")) {
      cout << "Right is pressed" << endl;
      this->msg_vel.angular.z -= vel_step;
      cout << msg_vel.angular.z << endl;
    }
    // increment y
    this->pos.y += button_height + width_padding;
    // backward
    if (cvui::button(frame, this->pos.x + button_width + width_padding,
                     this->pos.y, button_width, button_height, "Backward")) {
      cout << "Backward is pressed" << endl;
      this->msg_vel.linear.x -= vel_step;
      cout << msg_vel.linear.x << endl;
    }
    // increment y
    this->pos.y += button_height;
  }

  // Velocity Info
  void VelUI(int width_padding) {
    int single_window_width = (this->ui_width - width_padding * 3) / 2;
    int h = 45; // window title is 20, normal text height is 15

    // text starting point
    int title_height = 25;
    int text_start = single_window_width / 3;

    // linear vel window
    cvui::window(this->frame, this->pos.x, this->pos.y, single_window_width, h,
                 "Linear velocity:");

    // fill linear value to window
    cvui::printf(this->frame, this->pos.x + text_start,
                 this->pos.y + title_height, 0.4, 0xff0000, "%.2f",
                 this->msg_vel.linear.x);

    // angular vel window
    cvui::window(this->frame, this->pos.x + single_window_width + width_padding,
                 this->pos.y, single_window_width, h, "Angular velocity:");
    cvui::printf(this->frame,
                 this->pos.x + text_start + width_padding + single_window_width,
                 this->pos.y + title_height, 0.4, 0xff0000, "%.2f",
                 this->msg_vel.angular.z);

    // move pos.y down to bottom
    this->pos.y += h;
  }

  // Odom section
  void OdomUI(int width_padding) {
    int single_window_width = (this->ui_width - width_padding * 4) / 3;
    int h = 100; // window title is 20, normal text height is 15

    // text starting point
    int title_height = 60;
    int text_start = 0;
    float text_size = 1;
    // X
    cvui::window(this->frame, this->pos.x, this->pos.y, single_window_width, h,
                 "X");
    // x value
    cvui::printf(this->frame, this->pos.x + text_start,
                 this->pos.y + title_height, text_size, 0x00ff00, "%.2f",
                 this->x);

    // Y
    cvui::window(this->frame,
                 this->pos.x + (single_window_width + width_padding),
                 this->pos.y, single_window_width, h, "Y");
    // y value
    cvui::printf(
        this->frame,
        this->pos.x + text_start + (single_window_width + width_padding),
        this->pos.y + title_height, text_size, 0x00ff00, "%.2f", this->x);

    // Z
    cvui::window(this->frame,
                 this->pos.x + 2 * (single_window_width + width_padding),
                 this->pos.y, single_window_width, h, "Z");
    // z value
    cvui::printf(
        this->frame,
        this->pos.x + text_start + 2 * (single_window_width + width_padding),
        this->pos.y + title_height, text_size, 0x00ff00, "%.2f", this->x);

    // move pos.y down to bottom
    this->pos.y += h;
  }

  // Distance Button
  void DistanceUI(int padding) {
    int button_width = (this->ui_width - padding * 3) / 3,
        button_height = this->ui_height / 7;
    if (cvui::button(this->frame, this->pos.x, this->pos.y, button_width,
                     button_height, "Call")) {
      // once pressed init a trigger message
      std_srvs::Trigger msg_get;
      if (dis_client_.call(msg_get)) {
        response = msg_get.response.message;
      } else {
        response = "Distance Service Request Failed; Please try again.";
      }
    }

    // render window to show distance
    cvui::window(this->frame, this->pos.x + padding + button_width, this->pos.y,
                 (this->ui_width - padding * 3) - button_width, button_height,
                 "Distance in meters");
    // display distance number
    cvui::text(this->frame, this->pos.x + padding + button_width,
               this->pos.y + button_height - 50, response, 0.4);
  }
};