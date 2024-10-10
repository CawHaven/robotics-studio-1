#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
// #include <opencv2/highgui.hpp>

using namespace std::chrono_literals;

class System : public rclcpp::Node{
public:
    System() : Node("system_node"){
        laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&System::laserCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(50ms, std::bind(&System::node_main, this));
    }

private:
    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg){latest_laserscan_ = msg;}

    void laserProcessing(){};

    void node_main(){
        cv::Mat map_slam = cv::imread("/home/ovo/robotics-studio-1/map_slam.pgm", cv::IMREAD_GRAYSCALE);

        if(map_slam.empty()){
            RCLCPP_ERROR(this->get_logger(), "Could not load the ground truth map.");
            rclcpp::shutdown();
        }

        cv::threshold(map_slam, binary_map_, 200, 255, cv::THRESH_BINARY_INV);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_map_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Rect room_roi = cv::boundingRect(contours[0]);
        for (size_t i = 1; i < contours.size(); i++)
        {
            cv::Rect current_rect = cv::boundingRect(contours[i]);
            room_roi = room_roi | current_rect; // Expand to include all detected contours
        }

        // Crop the ground truth map to get only the actual room footprint
        auto map_cropped = map_slam(room_roi).clone();
        cv::cvtColor(map_cropped, map_cropped, cv::COLOR_GRAY2BGR); // Convert to color for visualization

        std::cout << "yo" << std::endl;


        // Show initial maps
        cv::imshow("Ground Truth Map", map_cropped);
        cv::imshow("Cropped Map", cv::Mat::zeros(map_cropped.size(), CV_8UC3));

        cv::imshow("Binary Map", binary_map_);

        cv::waitKey(20);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<sensor_msgs::msg::LaserScan> latest_laserscan_;

    cv::Mat binary_map_;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<System>());
    rclcpp::shutdown();
    return 0;
}
