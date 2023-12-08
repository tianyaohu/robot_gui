#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#define WINDOW_NAME "Frank's First CVUI"

using namespace std;

void vector2TextAndRec(cv::Mat &frame, vector<string> vec_str, int x, int y,
                       int rec_width) {

  // init rec (background)

  cout << "display frame info" << endl;
  // text spacing
  int text_spacing = 15;
  int padding = 0;
  for (const std::string &content : vec_str) {
    cvui::text(frame, x, y + padding, content, 0.4);
    padding += text_spacing;
    // cout << content << endl;
  }
  cvui::rect(frame, x, y, rec_width, padding, 0xff0000);
}

void group(cv::Mat &frame, int x, int y, int width, int height) {
  int padding = 5, w = (width - padding) / 4, h = (height - 15 - padding) / 2;
  cv::Point pos(x + padding, y + 5);

  cvui::text(frame, pos.x, pos.y, "Group title");
  pos.y += 15;

  vector<string> stringVector;

  // add string to vector;
  stringVector.push_back("hello");
  stringVector.push_back("world");
  stringVector.push_back("C++");
  stringVector.push_back("Vectors");
  stringVector.push_back("hello");
  stringVector.push_back("world");
  stringVector.push_back("C++");
  stringVector.push_back("Vectors");
  stringVector.push_back("hello");
  stringVector.push_back("world");
  stringVector.push_back("C++");
  stringVector.push_back("Vectors");
  stringVector.push_back("hello");
  stringVector.push_back("world");
  stringVector.push_back("C++");
  stringVector.push_back("Vectors");

  cvui::window(frame, pos.x, pos.y, width - padding * 2,
               15 + stringVector.size() * 15, "Something");
  vector2TextAndRec(frame, stringVector, pos.x + 2, pos.y + 20,
                    (width - padding * 2 - 5));
  //   cvui::text(frame, pos.x + 2, pos.y + 20, "this is a line of text", 0.4);

  pos.y += stringVector.size() * 20;

  cvui::window(frame, pos.x, pos.y, w / 3 - padding, h, "Some");
  cvui::text(frame, pos.x + 25, pos.y + 60, "65", 1.1);
  pos.x += w / 3;

  cvui::window(frame, pos.x, pos.y, w / 3 - padding, h, "Info");
  cvui::text(frame, pos.x + 25, pos.y + 60, "30", 1.1);
  pos.x += w / 3;

  cvui::window(frame, pos.x, pos.y, w / 3 - padding, h, "Here");
  cvui::text(frame, pos.x + 25, pos.y + 60, "70", 1.1);
  pos.x += w / 3;

  cvui::window(frame, pos.x, pos.y, w - padding, h, "And");
  cvui::rect(frame, pos.x + 2, pos.y + 22, w - padding - 5, h - padding - 20,
             0xff0000);
  pos.x += w;

  cvui::window(frame, pos.x, pos.y, w - padding, h, "Here");
  cvui::rect(frame, pos.x + 2, pos.y + 22, w - padding - 5, h - padding - 20,
             0xff0000);
  pos.x += w;

  cvui::window(frame, pos.x, pos.y, w - padding, h, "More info");
  cvui::rect(frame, pos.x + 2, pos.y + 22, w - padding - 5, h - padding - 20,
             0xff0000);
  pos.x += w;
}

int main(int argc, const char *argv[]) {
  int height = 220, spacing = 10;
  cv::Mat frame = cv::Mat(height * 3, 500, CV_8UC3);

  // Init cvui and tell it to create a OpenCV window, i.e.
  // cv::namedWindow(WINDOW_NAME).
  cvui::init(WINDOW_NAME);

  while (true) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // Render three groups of components.
    group(frame, 0, 0, frame.cols, height - spacing);

    // vector2TextAndRec(frame, stringVector, 20, 100);
    // group(frame, 0, height, frame.cols, height - spacing);
    // group(frame, 0, height * 2, frame.cols, height - spacing);

    // This function must be called *AFTER* all UI components. It does
    // all the behind the scenes magic to handle mouse clicks, etc.
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
  }

  return 0;
}

//   // height for each group
//   // - info group
//   int height_gen_info = 200;
//   // - control group
//   int height_teleop_button = 30, height_teleop = height_teleop_button * 3;
//   // - velocity info
//   int height_vel_info = 50;
//   // - pos info
//   int height_pos_info = 150;
//   // - distance info
//   int height_dis = 150 int height = 220, spacing = 10;