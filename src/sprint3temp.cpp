#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class System : public rclcpp::Node{
public:
    System() : Node("system_node"){
        laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&System::laserCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&System::odomCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cylinder_marker", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&System::node_main, this));
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Received a laser scan message.");

        // Reset the laser scan map to clear old data
        laser_scan_map_ = cv::Mat::zeros(ground_truth_map_cropped_.size(), CV_8UC3);

        try
        {
            // Get the transformation from the laser frame to the map frame
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform("map", msg->header.frame_id, rclcpp::Time(0));

            // Extract the yaw angle from the quaternion
            tf2::Quaternion q(
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            float angle = msg->angle_min;

            for (size_t i = 0; i < msg->ranges.size(); ++i)
            {
                float range = msg->ranges[i];
                if (range > msg->range_min && range < msg->range_max)
                {
                    // Convert the laser scan point to the world coordinates (using the transform rotation)
                    float x = range * cos(angle);
                    float y = range * sin(angle);

                    // Apply the transformation to get the coordinates in the map frame
                    float map_x = transform_stamped.transform.translation.x + (x * cos(yaw) - y * sin(yaw));
                    float map_y = transform_stamped.transform.translation.y + (x * sin(yaw) + y * cos(yaw));

                    // Convert (map_x, map_y) in world coordinates to pixel coordinates on the laser scan map
                    int pixel_x = static_cast<int>((map_x - map_origin_.first) / map_resolution_);
                    int pixel_y = static_cast<int>((-map_y - map_origin_.second) / map_resolution_);

                    if (pixel_x >= 0 && pixel_x < laser_scan_map_.cols && pixel_y >= 0 && pixel_y < laser_scan_map_.rows)
                    {
                        cv::circle(laser_scan_map_, cv::Point(pixel_x, pixel_y), 1, cv::Scalar(255, 255, 255), -1);

                        // Check if this point is an obstacle on the ground truth map
                        if (ground_truth_map_cropped_.at<cv::Vec3b>(pixel_y, pixel_x) == cv::Vec3b(255, 255, 255))
                        {
                            // Check if the point is at least 0.5 meters away from any other black part of the ground truth map
                            if (is_clear_around(pixel_x, pixel_y))
                            {
                                // If it is detected as an obstacle by the laser scan but not present on the ground truth, mark it as a potential cylinder
                                if (!detected_cylinder_)
                                {
                                    detected_cylinder_ = {pixel_x, pixel_y};
                                    draw_cylinder_marker(pixel_x, pixel_y);

                                    // Print the global coordinates of the detected cylinder
                                    RCLCPP_INFO(this->get_logger(), "Cylinder detected and marked at pixel coordinates (%d, %d) on the map.", pixel_x, pixel_y);
                                    RCLCPP_INFO(this->get_logger(), "Global coordinates of detected cylinder: x = %.2f, y = %.2f", map_x, map_y);
                                }
                            }
                        }
                    }
                }
                angle += msg->angle_increment;
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }

        // Display updated laser scan map
        cv::imshow("Laser Scan Map", laser_scan_map_);
        cv::waitKey(1);
    }

    bool is_clear_around(int pixel_x, int pixel_y)
    {
        // Check if there is any black part within 0.5m radius in the ground truth map
        int radius_in_pixels = static_cast<int>(0.5 / map_resolution_); // Convert 0.5m to pixels

        for (int dy = -radius_in_pixels; dy <= radius_in_pixels; ++dy)
        {
            for (int dx = -radius_in_pixels; dx <= radius_in_pixels; ++dx)
            {
                int new_x = pixel_x + dx;
                int new_y = pixel_y + dy;

                if (new_x >= 0 && new_x < ground_truth_map_cropped_.cols && new_y >= 0 && new_y < ground_truth_map_cropped_.rows)
                {
                    if (ground_truth_map_cropped_.at<cv::Vec3b>(new_y, new_x) == cv::Vec3b(0, 0, 0))
                    {
                        return false; // There is an obstacle too close
                    }
                }
            }
        }
        return true;
    }

    /**
     * @brief Draws a marker to indicate the detected cylinder on both laser scan and ground truth maps.
     * @param pixel_x X coordinate of the detected cylinder in pixels.
     * @param pixel_y Y coordinate of the detected cylinder in pixels.
     */
    void draw_cylinder_marker(int pixel_x, int pixel_y)
    {
        // Draw a marker on both laser scan map and ground truth map
        cv::circle(laser_scan_map_, cv::Point(pixel_x, pixel_y), 5, cv::Scalar(0, 0, 255), -1);
        cv::circle(ground_truth_map_cropped_, cv::Point(pixel_x, pixel_y), 5, cv::Scalar(0, 0, 255), -1);
    }

    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg){latest_odom_ = msg;}

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        // Parameters
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    image.at<uchar>(y, x) = 255;
                }
            }
        }
        return image;
    }

    void cylinder_visualisation(double x, double y){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";  // The frame of the detected object
        marker.header.stamp = this->now();
        marker.ns = "cylinder";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the cylinder
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.5;  // Assuming the object is tall enough for the laser to see
        marker.pose.orientation.w = 1.0;

        // Set the dimensions of the marker
        marker.scale.x = 0.30;  // Diameter of the cylinder
        marker.scale.y = 0.30;
        marker.scale.z = 1.0;   // Height of the object

        // Set color to make it easily visible
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;   // Semi-transparent

        marker.lifetime = rclcpp::Duration::from_seconds(0);  // Marker remains indefinitely

        marker_pub_->publish(marker);
    }



    void node_main(){
        cv::Mat map_slam = cv::imread("/home/ovo/robotics-studio-1/map_slam.pgm", cv::IMREAD_GRAYSCALE);

        if(map_slam.empty()){
            RCLCPP_ERROR(this->get_logger(), "Could not load the ground truth map.");
            rclcpp::shutdown();
        }

        cv::threshold(map_slam, binary_map_, 200, 255, cv::THRESH_BINARY_INV);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_map_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


        std::cout << "yo" << std::endl;

        if(!(latest_laserscan_ == nullptr)){
            auto laser = laserScanToMat(latest_laserscan_);



            cv::waitKey(20);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::shared_ptr<nav_msgs::msg::Odometry> latest_odom_;

    std::shared_ptr<sensor_msgs::msg::LaserScan> latest_laserscan_;

    cv::Mat binary_map_, map_cropped, laser_scan_map_;

    cv::Mat ground_truth_map_, ground_truth_map_cropped_, binary_map_, laser_scan_map_; /**< OpenCV matrices for the maps */
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_; /**< Subscription to laser scan data */
    std::optional<std::pair<int, int>> detected_cylinder_; /**< Coordinates of detected cylinder (if any) */

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; /**< Buffer for storing transforms */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; /**< Listener for transform data */

    double map_resolution_; /**< Resolution of the map in meters/pixel */
    std::pair<double, double> map_origin_; /**< Origin of the map in meters (bottom-left corner) */

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<System>());
    rclcpp::shutdown();
    return 0;
}
