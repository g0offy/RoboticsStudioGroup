#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

// Define the MapProcessorNode class which inherits from rclcpp::Node
class MapProcessorNode : public rclcpp::Node
{
public:
    // Constructor initializes the node and subscriptions
    MapProcessorNode()
    : Node("map_processor_node") // Initialize node with the name "map_processor_node"
    {
        // Subscribe to the /map topic to receive occupancy grid data
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1));

        // Subscribe to the /scan topic to receive laser scan data
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapProcessorNode::scanCallback, this, std::placeholders::_1));

        // Subscribe to the /odom topic to receive odometry data
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MapProcessorNode::odomCallback, this, std::placeholders::_1));

        // Create an OpenCV window to display the map
        cv::namedWindow(WINDOW1, cv::WINDOW_AUTOSIZE);

        // Set an initial pose for the robot
        setInitialPose(cv::Point(100, 100), 0.0); // Example position (100,100) and orientation (0 degrees)
    }

private:
    // Function to set the robot's initial pose (position and orientation)
    void setInitialPose(const cv::Point& position, double orientation)
    {
        robot_position_ = position; // Set robot's initial position
        robot_orientation_ = orientation; // Set robot's initial orientation
    }

    // Callback function for laser scan data
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::cout << "scanCallback" << std::endl;

        // Convert LaserScan to an OpenCV image (Image C)
        cv::Mat imageC = laserScanToMat(msg);

        // Display Image C for visual verification
        cv::imshow("Image C", imageC);
        cv::waitKey(1);

        // If Image B is already available, estimate the robot's yaw change using Image B and Image C
        if (!imageB_resized.empty()) {
            estimateRotation(imageB_resized, imageC); // Estimate rotation
            visualizeMatches(imageB_resized, imageC); // Visualize the feature matches between Image B and Image C
        }
    }

    // Callback function for odometry data
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::cout << "odomCallback" << std::endl;

        // Update the robot's position and orientation based on odometry data
        robot_position_.x = static_cast<int>((msg->pose.pose.position.x - origin_x) / map_scale_);
        robot_position_.y = static_cast<int>((msg->pose.pose.position.y - origin_y) / map_scale_);

        // Compute the yaw angle from the quaternion representation of the robot's orientation
        auto& q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        robot_orientation_ = std::atan2(siny_cosp, cosy_cosp);
    }

    // Function to convert LaserScan data to an OpenCV image
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        // Create a blank image (500x500)
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1); // Create a black image

        // Iterate over each laser scan range
        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                // Calculate angle and coordinates in image space
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    image.at<uchar>(y, x) = 255; // Set the pixel to white if within bounds
                }
            }
        }
        return image; // Return the generated image
    }

    // Callback function for map (occupancy grid) data
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        std::cout << "mapCallback" << std::endl;

        // Convert the occupancy grid to a binary image
        occupancyGridToImage(mapMsg);

        // Extract a section of the map around the robot's current position (Image A)
        int roi_x = std::max(0, std::min(robot_position_.x - roi_size_ / 2, m_MapColImage.cols - roi_size_));
        int roi_y = std::max(0, std::min(robot_position_.y - roi_size_ / 2, m_MapColImage.rows - roi_size_));
        int roi_width = std::min(roi_size_, m_MapColImage.cols - roi_x);
        int roi_height = std::min(roi_size_, m_MapColImage.rows - roi_y);

        // Ensure the region of interest (ROI) is within bounds
        if (roi_width > 0 && roi_height > 0) {
            // Extract and rotate the section of the map around the robot (Image A)
            cv::Rect roi(roi_x, roi_y, roi_width, roi_height);
            cv::Mat imageA = m_MapColImage(roi).clone(); // Extract the section of the map

            // Rotate Image A based on the robot's orientation
            cv::Mat imageA_rotated;
            cv::Point2f center(imageA.cols / 2.0, imageA.rows / 2.0);
            cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, robot_orientation_ * 180.0 / CV_PI, 1.0);
            cv::warpAffine(imageA, imageA_rotated, rotation_matrix, imageA.size());

            // Resize Image A for better visibility
            cv::Mat imageA_resized;
            cv::resize(imageA_rotated, imageA_resized, cv::Size(), 3.0, 3.0, cv::INTER_LINEAR); // Scale by 3x

            // Display Image A (resized and rotated) for verification
            cv::imshow("Image A", imageA_resized);

            // Apply Canny edge detection to Image A to create Image B
            cv::Mat imageB;
            cv::Canny(imageA_rotated, imageB, 100, 200); // Perform edge detection

            // Resize Image B to match the size of Image A
            cv::resize(imageB, imageB_resized, imageA_resized.size(), 0, 0, cv::INTER_LINEAR);

            // Display Image B (edges) for verification
            cv::imshow("Image B", imageB_resized);
        } else {
            std::cerr << "ROI is out of bounds, cannot extract Image A" << std::endl;
        }

        cv::waitKey(1);

        // Rotate and display the entire map image
        cv::Mat tmp_col_img = m_MapColImage.clone();
        cv::rotate(tmp_col_img, tmp_col_img, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::imshow(WINDOW1, tmp_col_img); // Display the rotated map
        cv::waitKey(1);
    }

    // Convert occupancy grid data into an OpenCV image
    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    {
        int grid_data;
        unsigned int row, col, val;

        // Create a blank grayscale image based on the size of the occupancy grid
        m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        std::cout << "DataParse started for map: " << grid->header.stamp.sec << " Dim: " << grid->info.height << "x" << grid->info.width << std::endl;

        // Iterate through the occupancy grid and set pixel values accordingly
        for (row = 0; row < grid->info.height; row++) {
            for (col = 0; col < grid->info.width; col++) {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1) {
                    val = 255 - (255 * grid_data) / 100; // Convert occupancy value to grayscale
                    val = (val == 0) ? 255 : 0; // Mark free cells as white
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val; // Set pixel value
                } else {
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0; // Unknown cells are black
                }
            }
        }

        // Store map properties such as scale and origin
        map_scale_ = grid->info.resolution;
        origin_x = grid->info.origin.position.x;
        origin_y = grid->info.origin.position.y;
        size_x = grid->info.width;
        size_y = grid->info.height;

        // Perform morphological erosion to refine the binary map image
        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                                    0, 1, 0,
                                    0, 0, 0);
        cv::erode(m_temp_img, m_MapBinImage, kernel); // Apply erosion

        // Convert the binary map image to a 3-channel image (color image)
        m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
        cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR); // Convert to color image

        std::cout << "Occupancy grid map converted to a binary image\n";
    }

    // Estimate the robot's yaw change by detecting and matching features between Image B and Image C
    void estimateRotation(const cv::Mat& imageB, const cv::Mat& imageC) {
        // Detect and match features between Image B and Image C
        std::vector<cv::Point2f> srcPoints, dstPoints;
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        std::vector<cv::DMatch> goodMatches;
        detectAndMatchFeatures(imageB, imageC, srcPoints, dstPoints, keypoints1, keypoints2, goodMatches);

        // Ensure sufficient points are detected for transformation estimation
        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try {
            // Estimate the transformation matrix (affine transformation) between the two images
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            } else {
                // Extract the rotation angle from the transformation matrix
                double angle_difference = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference = angle_difference * 180.0 / CV_PI; // Convert radians to degrees
                robot_orientation_ += angle_difference; // Update the robot's orientation
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference);
                RCLCPP_INFO(this->get_logger(), "Updated robot orientation: %f degrees", robot_orientation_);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }

    // Detect and match features between two images
    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints,
                                std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2,
                                std::vector<cv::DMatch>& goodMatches) {
        cv::Ptr<cv::ORB> orb = cv::ORB::create(); // Create ORB feature detector
        cv::Mat descriptors1, descriptors2;

        // Detect and compute keypoints and descriptors for both images
        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        // Use brute-force matcher to find matching features
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches); // Match descriptors

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Keep the top 15% of the best matches
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);
        goodMatches.assign(matches.begin(), matches.begin() + numGoodMatches);

        // Store the corresponding points for further transformation estimation
        for (const auto& match : goodMatches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    // Visualize feature matches between two images
    void visualizeMatches(const cv::Mat& img1, const cv::Mat& img2) {
        // Detect and match features between the two images
        std::vector<cv::Point2f> srcPoints, dstPoints;
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        std::vector<cv::DMatch> goodMatches;
        detectAndMatchFeatures(img1, img2, srcPoints, dstPoints, keypoints1, keypoints2, goodMatches);

        // Draw the matches between the two images
        cv::Mat img_matches;
        cv::drawMatches(img1, keypoints1, img2, keypoints2, goodMatches, img_matches);

        // Display the feature matches
        cv::imshow("Feature Matches", img_matches);
        cv::waitKey(1);
    }

    // ROS subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_; // For occupancy grid
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_; // For laser scans
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_; // For odometry data

    // OpenCV images
    cv::Mat m_temp_img, m_MapBinImage, m_MapColImage; // Images for map processing
    cv::Mat imageB_resized; // Resized Image B for rotation estimation
    cv::Size laser_scan_size_; // Size of the laser scan image

    // Robot position and orientation
    cv::Point robot_position_; // Robot position in map coordinates
    double robot_orientation_; // Robot orientation (yaw angle)
    
    // Region of interest (ROI) size
    int roi_size_ = 50; // Size of the region of interest (ROI) around the robot

    // Window name for displaying the map
    const std::string WINDOW1 = "Map Window";

    // Map properties
    double map_scale_; // Scale of the map (meters per pixel)
    double origin_x, origin_y; // Origin of the map (in meters)
    int size_x, size_y; // Size of the map (in pixels)
};

// Main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // Initialize the ROS 2 node
    rclcpp::spin(std::make_shared<MapProcessorNode>()); // Spin the node to process callbacks
    rclcpp::shutdown(); // Shutdown the node
    return 0; // Exit the program
}
