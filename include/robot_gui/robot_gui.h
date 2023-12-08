#include <iostream>
#include <ros/ros.h>
#include <sstream>

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#define WINDOW_NAME "Frank's Robot GUI"

using namespace std;

class RobotGUI {
public:
  // constructor
  RobotGUI()
      : vec_info({"Start1", "Start2", "Start2", "Start2", "Start2", "Start2",
                  "Start2", "Start2", "Start2", "Start2", "Start2"}),
        linear_x(0), angular_z(0) {
    cout << "RobotGUI Constructor is called" << endl;
  };
  // destructor
  ~RobotGUI() { cout << "RobotGUI Destructor is called" << endl; };

  void DisplayUI() {
    cout << "DisplayUI called" << endl;
    // init a padding paramter
    int padding = 10;

    // init frame and window
    init_UI_frame(800, 500);
    cvui::init(WINDOW_NAME);

    // while esc is not pressed
    while (cv::waitKey(20) != 27) {
      // reset pos
      this->pos = cv::Point(padding, padding);
      // Fill the frame with a nice color
      frame = cv::Scalar(49, 52, 49);

      // info ui
      this->InfoUI(padding);

      // add padding
      this->pos.y += 30;

      // control section
      // pass

      // velocity section
      this->VelUI(padding);

      // update ui and show window
      cvui::update();
      cv::imshow(WINDOW_NAME, this->frame);

      cout << "within while loop" << endl;
    }
    cout << "DisplayUI termminated" << endl;
  }

private:
  ros::NodeHandle *m_ros_node_object;

  ros::Subscriber info_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher vel_pub_;

  // UI contents:
  // - general info
  vector<string> vec_info;

  // velocity linear x
  float linear_x;
  float angular_z;

  //  - Robot Position
  float x;
  float y;
  float z;
  // distance
  float dis;

  // UI frame
  int ui_height;
  int ui_width;
  cv::Mat frame;
  // init point for ui alignment
  cv::Point pos;
  // section dimensions

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
  }

  void VelUI(int width_padding) {
    int single_window_width = (this->ui_width - width_padding * 3) / 2;
    int h = 45; // window title is 20, normal text height is 15

    // text starting point
    int title_height = 25;
    int text_start = single_window_width / 3;

    // convert a floating number to string with 2 decimal precision rounding
    auto float2String = [](float f) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << f;
      return oss.str();
    };
    // linear vel window
    cvui::window(this->frame, this->pos.x, this->pos.y, single_window_width, h,
                 "Linear velocity:");
    // fill linear value to window
    cvui::text(this->frame, this->pos.x + text_start,
               this->pos.y + title_height,
               float2String(this->linear_x) + " m/sec", 0.4, 0xff0000);

    // angular vel window
    cvui::window(this->frame, this->pos.x + single_window_width + width_padding,
                 this->pos.y, single_window_width, h, "Angular velocity:");
    cvui::text(this->frame,
               this->pos.x + text_start + width_padding + single_window_width,
               this->pos.y + title_height,
               float2String(this->linear_x) + " rod/sec", 0.4, 0xff0000);
  }

  // Velocity Info
};
