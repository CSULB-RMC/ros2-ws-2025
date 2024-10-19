#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
using namespace std;

class Bmo : public rclcpp::Node
{
public:
  void changeFrame(int x, int y, int color) {
    frame[x][y] = color;
  }

  void render() {
    ofstream console1("/dev/tty1");
    ofstream console2("/dev/tty2");

    //string line = "\033c";
    string line = "\n";
    for(int i = 0; i < height; i++) {
      for(int j = 0; j < width; j++) {
          line += "\033[48;5;" + to_string(frame[i][j]) + "m \033[0m";
      }
      //line += "\n";
    }
    consoleFrame = (consoleFrame + 1) % 2;
    if(consoleFrame) {
    	cout.rdbuf(console2.rdbuf());
	    system("chvt 1");
    } else {
      cout.rdbuf(console1.rdbuf());
      system("chvt 2");
    }
    cout << line << flush;
    console1.close();
    console2.close();
  }

  void play() {
    ifstream file("src/bmo_ros/frameSets/" + set + ".txt");

    if(!file.is_open()) {
        cerr << "Error opening file" << endl;
    }

    string temp_frame; 
    string raw_pixel;
    string raw_value;
    vector<int> temp_pixel;

    vector<vector<int>> clearing_array;

    //one frame
    while(getline(file, temp_frame)) {
        stringstream pixels(temp_frame);

        //one pixel
        while(getline(pixels, raw_pixel, '|')) {
            stringstream values(raw_pixel);
            temp_pixel.clear();
            //one value of that pixel coordinate and color
            while(getline(values, raw_value, ',')) {
                temp_pixel.push_back(stoi(raw_value));
            }
            changeFrame(temp_pixel[0], temp_pixel[1], temp_pixel[2]);
            clearing_array.push_back(temp_pixel);
        }
        render();
       	usleep(50000); 

        while(!clearing_array.empty()) {
            changeFrame(clearing_array[0][0], clearing_array[0][1], bg_color);
            clearing_array.erase(clearing_array.begin());
        }
        usleep(600000);
    }
    file.close();
  }

  Bmo()
  : Node("bmo")
  {
    for(int i = 0; i < this->height; i++) {
        for(int j = 0; j < this->width; j++) {
            this->frame[i][j] = this->bg_color;
        }
    }

    auto timer_callback =
      [this]() -> void {  
        this->count_ += 1;
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);

    auto change_animation =
      [this](std_msgs::msg::Int32::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "Changing Animation");
        
        switch(msg->data) {
          case 0:
            this->set = "loading";
            break;
          case 1:
            this->set = "blink";
            break;
          case 2:
            this->set = "lookMidToTopRight";
            break;
          case 3:
            this->set = "lookTopRightToMid";
            break;
          default:
            this->set = "loading";
        }
      };
    subscription_ =
      this->create_subscription<std_msgs::msg::Int32>("bmo_animation", 10, change_animation);
    play();
    play();
    play();
    play();
    play();
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

  int width = 100;
  int height = 30;
  int frame[30][100]; // for speed at compile time you need to put y, x here
  int consoleFrame = 0;
  int bg_color = 6;
  string set = "loading";

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bmo>());
  rclcpp::shutdown();
  return 0;
}