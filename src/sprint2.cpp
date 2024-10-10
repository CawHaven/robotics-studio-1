#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <chrono>

using namespace std::chrono_literals;

// Input: Odometry of bot, Lidar Array, World map
// Output: Angular Difference
class System : public rclcpp::Node{
public:
    System() : Node("system_node"){
        laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&System::laserCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&System::odomCallback, this, std::placeholders::_1));
        // world_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        //     "/map_server", 10, std::bind(&System::worldmapCallback, this, std::placeholders::_1));

        rotation_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/rotation", 10);

        publish_timer_ = this->create_wall_timer(50ms, std::bind(&System::publishCallback, this));

    }

private:
    // Receive lidar
    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg){latest_laserscan_ = msg;}

    // Lidar to Image
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

    // Receive Odometry Position
    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg){latest_odom_ = msg;}

    // Receive world map data
    void worldmapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg){latest_world_map_ = msg;}

    // Extract image from world map around robot & Return Extracted Image
    cv::Mat extractSubImageAsMat(const nav_msgs::msg::OccupancyGrid &msg, float x, float y, float side_length){
        const auto &info = msg.info;
        const float resolution = info.resolution;
        const auto &origin = info.origin.position;

        // Map dimensions in pixels
        const int width = info.width;
        const int height = info.height;

        // Check if data is valid
        if (msg.data.empty() || width <= 0 || height <= 0 || resolution <= 0) {
            std::cerr << "Invalid map data or dimensions." << std::endl;
            return cv::Mat(); // Return an empty cv::Mat
        }

        // Convert (x, y) coordinates to map pixel coordinates
        int x_pixel = static_cast<int>((x - origin.x) / resolution);
        int y_pixel = static_cast<int>((y - origin.y) / resolution);

        // Calculate half side length in pixels
        int half_side = static_cast<int>(side_length / (2 * resolution));

        // Determine the bounding box
        int x_min = std::max(0, x_pixel - half_side);
        int y_min = std::max(0, y_pixel - half_side);
        int x_max = std::min(width - 1, x_pixel + half_side);
        int y_max = std::min(height - 1, y_pixel + half_side);

        // Dimensions of the sub-image
        int sub_image_width = x_max - x_min + 1;
        int sub_image_height = y_max - y_min + 1;

        // int sub_image_width = 1;
        // int sub_image_height = 1;

        // Create a cv::Mat to hold the sub-image data
        cv::Mat sub_image(sub_image_height, sub_image_width, CV_8SC1);

        // Fill the cv::Mat with data from the map
        for (int y = y_min; y <= y_max; ++y) {
            for (int x = x_min; x <= x_max; ++x) {
                // Compute the linear index for the map
                int index = y * width + x;
                if (index < 0 || index >= static_cast<int>(msg.data.size())) {
                    std::cerr << "Index out of bounds." << std::endl;
                    return cv::Mat(); // Return an empty cv::Mat
                }
                // Fill the cv::Mat
                sub_image.at<signed char>(y - y_min, x - x_min) = msg.data[index];
            }
        }

        return sub_image;
    }

    cv::Mat extractSubImageAsMat(const cv::Mat &map, float x, float y, float side_length, float resolution, const cv::Point2f &origin) {
        // Map dimensions in pixels
        const int width = map.cols;
        const int height = map.rows;

        // Check if data is valid
        if (map.empty() || width <= 0 || height <= 0 || resolution <= 0) {
            std::cerr << "Invalid map data or dimensions." << std::endl;
            return cv::Mat(); // Return an empty cv::Mat
        }

        // Convert (x, y) coordinates to map pixel coordinates
        int x_pixel = static_cast<int>((x - origin.x) / resolution);
        int y_pixel = static_cast<int>((y - origin.y) / resolution);

        // Calculate half side length in pixels
        int half_side = static_cast<int>(side_length / (2 * resolution));

        // Determine the bounding box
        int x_min = std::max(0, x_pixel - half_side);
        int y_min = std::max(0, y_pixel - half_side);
        int x_max = std::min(width - 1, x_pixel + half_side);
        int y_max = std::min(height - 1, y_pixel + half_side);

        // Dimensions of the sub-image
        int sub_image_width = x_max - x_min + 1;
        int sub_image_height = y_max - y_min + 1;

        // Create a cv::Mat to hold the sub-image data
        cv::Mat sub_image(sub_image_height, sub_image_width, map.type());

        // Fill the cv::Mat with data from the map
        for (int y = y_min; y <= y_max; ++y) {
            for (int x = x_min; x <= x_max; ++x) {
                // Copy the data from the map to the sub-image
                sub_image.at<cv::Vec<uchar, 1>>(y - y_min, x - x_min) = map.at<cv::Vec<uchar, 1>>(y, x);
            }
        }

        return sub_image;
    }

    // Edge detection of smaller image & Return Extracted Edge Image
    void calculateYawChange(cv::Mat first_image_, cv::Mat second_image_, double& angle_difference_) {
        // Detect and match features between the first and second images
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            } else {
                // Extract the rotation angle from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = angle_difference_ * 180.0 / CV_PI;
                // RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }

    }

    // Compare 2 Images and return angular difference & publish it
    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2, std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Determine the number of top matches to keep (30% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : matches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    cv::Mat detect_edges(const cv::Mat& input_image) {
        if (input_image.empty()) {
            throw std::invalid_argument("Input image is empty.");
        }

        cv::Mat gray_image;

        // Check if input image is already grayscale
        if (input_image.channels() == 1) {
            gray_image = input_image.clone();
        } else if (input_image.channels() == 3) {
            // Convert to grayscale
            cv::cvtColor(input_image, gray_image, cv::COLOR_BGR2GRAY);
        } else {
            throw std::invalid_argument("Unsupported number of channels in input image.");
        }

        // Apply Gaussian Blur
        cv::Mat blurred_image;
        cv::GaussianBlur(gray_image, blurred_image, cv::Size(5, 5), 3);

        // Edge detection using Canny
        cv::Mat edges;
        double lower_threshold = 5;
        double upper_threshold = 10;
        cv::Canny(blurred_image, edges, lower_threshold, upper_threshold);

        return edges;
    }

    enum InterpolationMethod {
        NEAREST_NEIGHBOR,
        LINEAR
    };

    cv::Mat scale_image(const cv::Mat& input_image, double scale_factor, InterpolationMethod method) {
        if (input_image.empty()) {
            std::cerr << "Input image is empty." << std::endl;
            return cv::Mat();
        }

        // Determine interpolation method
        int interpolation;
        switch (method) {
            case NEAREST_NEIGHBOR:
                interpolation = cv::INTER_NEAREST;
                break;
            case LINEAR:
                interpolation = cv::INTER_LINEAR;
                break;
            default:
                std::cerr << "Unknown interpolation method." << std::endl;
                return cv::Mat();
        }

        // Calculate new size
        cv::Size new_size(static_cast<int>(input_image.cols * scale_factor),
                          static_cast<int>(input_image.rows * scale_factor));

        // Resize the image
        cv::Mat scaled_image;
        cv::resize(input_image, scaled_image, new_size, 0, 0, interpolation);

        return scaled_image;
    }

    cv::Mat applyGaussianBlur(const cv::Mat& inputImage, int kernelSize, double sigmaX) {
        // Check if the input image is empty
        if (inputImage.empty()) {
            std::cerr << "Input image is empty!" << std::endl;
            return cv::Mat();
        }

        // Ensure kernel size is odd and greater than 1
        if (kernelSize % 2 == 0 || kernelSize <= 1) {
            std::cerr << "Kernel size must be an odd number greater than 1!" << std::endl;
            return cv::Mat();
        }

        // Output image
        cv::Mat outputImage;

        // Apply Gaussian blur
        cv::GaussianBlur(inputImage, outputImage, cv::Size(kernelSize, kernelSize), sigmaX);

        return outputImage;
    }

    double updateAndComputeRMS(double newDiff) {
        // Static variables to maintain state across function calls
        static double sumOfSquares = 0.0;
        static int count = 0;

        // Update the sum of squares and count
        sumOfSquares += newDiff * newDiff;
        count++;

        // Calculate the RMS error
        if (count == 0) {
            return 0.0; // Avoid division by zero
        }
        return std::sqrt(sumOfSquares / count);
    }

    std::vector<double> values;
    double movingaverage(double value){
        values.push_back(value);
        if(!values.empty()){
            if(values.size()<50){
                for(int i = 0; i < 50; i++){
                    values.push_back(0);
                }
            }
            if(values.size() > 1000){
                values.erase(values.begin());
            }
            double total = 0;
            for(auto val: values){
                total += val;
            }
            return total/values.size();
        }
        return 0;
    }

    /**     SLO 3.5
     * Use ROS2 nav2_amcl package to localise the robot
     */

    /**     SLO 3.6
     * Set the initial robot pose to a known location of the map
     * Extract section of the map around the robot (let's call this Image A)
     * Extract edges from Image A using opencv and create and edge image (let's call this Image B)
     * Receive a laser scan and convert it to an image (let's call this Image C)
     * Similar to WK5 lab activity use Image B and Image C to estimate the rotation of robot relative to the map
     * Using odometry, propagate the robot to the new location
     */

    /**     SLO 3.7
     * Compare the localisation accuracies against the ground truth
     */


    void publishCallback(){
        if(!((latest_laserscan_ == nullptr)||(latest_odom_ == nullptr))){
            float robot_pos_x = latest_odom_->pose.pose.position.x;
            float robot_pos_y = latest_odom_->pose.pose.position.y;

            auto worldmap = scale_image(cv::imread("/home/ovo/robotics-studio-1/src/my_map.pgm", cv::IMREAD_UNCHANGED), 5, LINEAR);
            cv::Point2f point;
            point.x = -2.95;
            point.y = -2.58;
            auto cropped_image = extractSubImageAsMat(worldmap, robot_pos_x, robot_pos_y, 40, 0.05, point);

            cv::imshow("Cropped", detect_edges(cropped_image));
            cv::resizeWindow("Cropped", 600, 600);



            // cv::imshow("World Map", worldmap);

            // cv::imshow("Scaled Map", worldmap);

            double angle_diff;

            auto laser = laserScanToMat(latest_laserscan_);

            cv::imshow("Lidar", laser);

            auto blurlaser = applyGaussianBlur(laser, 5, 1);

            cv::imshow("LidarImage", detect_edges(blurlaser));
            // cv::resizeWindow("LidarImage", 800, 600);

            // std::cout << latest_laserscan_->ranges.size() << std::endl;

            calculateYawChange(detect_edges(cropped_image), detect_edges(blurlaser), angle_diff);

            std::cout << "Angle Diff: " << movingaverage(angle_diff);
            std::cout << "      Estimated Error RMS: " << updateAndComputeRMS(movingaverage(angle_diff)) << std::endl;

            // std::cout << angle_diff << std::endl;


            // cv::imshow("Lidar", laserScanToMat(latest_laserscan_));
            cv::waitKey(10);
        }

    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr world_map_sub_;
    std::shared_ptr<sensor_msgs::msg::LaserScan> latest_laserscan_;
    std::shared_ptr<nav_msgs::msg::Odometry> latest_odom_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> latest_world_map_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rotation_pub_;

    rclcpp::TimerBase::SharedPtr publish_timer_;
    
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<System>());
    rclcpp::shutdown();
    return 0;
}
