#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode()
        : Node("map_processor_node"), robot_x_(-2.0), robot_y_(-0.5), robot_theta_(0)
    {
        // Subscribers
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1));

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapProcessorNode::scanCallback, this, std::placeholders::_1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MapProcessorNode::odomCallback, this, std::placeholders::_1));

        // Publisher
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // OpenCV Windows
        cv::namedWindow(WINDOW1, cv::WINDOW_AUTOSIZE); // Map from occupancy grid
        cv::namedWindow(WINDOW2, cv::WINDOW_AUTOSIZE); // Map section
        cv::namedWindow(WINDOW3, cv::WINDOW_AUTOSIZE); // Laser scan image
        cv::namedWindow(WINDOW4, cv::WINDOW_AUTOSIZE); // Feature detection
    }

private:
    // Callback for LaserScan messages
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::cout << "scanCallback" << std::endl;

        // Convert LaserScan to image
        cv::Mat scan_image = laserScanToMat(msg);

        // Check if scan_image is empty
        if (scan_image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Laser scan image is empty. Skipping processing.");
            return;
        }

        // Display the raw laser scan image without recoloring or intensity modifications
        cv::imshow(WINDOW3, scan_image);
        cv::waitKey(1);

        // Clone the scan image for processing
        scan_image_ = scan_image.clone();

        // If the map section is available, perform further processing
        if (!map_section_.empty())
        {
            calculateYawChange();
            visualizeMatches(map_section_, scan_image_);
            cv::waitKey(1);
        }
    }

    // Callback for OccupancyGrid messages
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        std::cout << "mapCallback" << std::endl;

        // Convert OccupancyGrid to image
        occupancyGridToImage(mapMsg);

        // Update and display the full map image
        cv::imshow(WINDOW1, map_image_);
        cv::waitKey(1);

        // After updating the map, attempt to update the map section based on the current robot pose
        updateMapSection();
    }

    // Callback for Odometry messages
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update robot's position
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        // Convert quaternion to Euler angles for the orientation
        auto q = msg->pose.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch;
        mat.getRPY(roll, pitch, robot_theta_); // Update robot_theta_ directly

        // After updating the robot's pose, attempt to update the map section
        updateMapSection();
    }

    // Function to convert LaserScan to OpenCV Mat
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {
        int img_size = 200;
        float max_range = scan->range_max;
        // Create a black image (all values initialized to 0)
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++)
        {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max)
            {
                // Calculate x and y based on the range and angle of the scan
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle) * img_size / (2 * max_range)) + img_size / 2);
                int y = static_cast<int>((range * sin(angle) * img_size / (2 * max_range)) + img_size / 2);

                // Ensure the calculated point is within the image bounds
                if (x >= 0 && x < img_size && y >= 0 && y < img_size)
                {
                    // Assign brightness based on proximity: closer points are brighter
                    int intensity = static_cast<int>((1 - range / max_range) * 255);
                    image.at<uchar>(y, x) = intensity; // Set pixel brightness
                    
                }
            }
        }

        // Do not modify the colors or intensities any further
        return image; // Return the image with a black background
    }

    // Function to convert OccupancyGrid to OpenCV Mat
    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    {
        int grid_data;
        unsigned int row, col, val;

        m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        std::cout << "DataParse started for map: " << grid->header.stamp.sec << " Dim: " << grid->info.height << "x" << grid->info.width << std::endl;

        // Use a larger grid_spacing value to make the snapping less strict
        const int grid_spacing = 1; // Lower value to allow more flexibility in points alignment

        for (row = 0; row < grid->info.height; row++)
        {
            for (col = 0; col < grid->info.width; col++)
            {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1)
                {
                    val = 255 - (255 * grid_data) / 100;
                    val = (val == 0) ? 255 : 0;

                    // Snap points to grid using a smaller grid spacing value
                    int snapped_row = (row / grid_spacing) * grid_spacing;
                    int snapped_col = (col / grid_spacing) * grid_spacing;

                    m_temp_img.at<uchar>(grid->info.height - snapped_row - 1, snapped_col) = val;
                }
                else
                {
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                }
            }
        }

        map_scale_ = grid->info.resolution;
        origin_x = grid->info.origin.position.x;
        origin_y = grid->info.origin.position.y;
        size_x = grid->info.width;
        size_y = grid->info.height;

        // Erode the image to reduce noise
        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                          0, 1, 0,
                          0, 0, 0);
        cv::erode(m_temp_img, m_MapBinImage, kernel);

        // Convert binary image to color for visualization
        m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
        cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);

        std::cout << "Occupancy grid map converted to a binary image with more flexible points.\n";

        // Store the full map image
        map_image_ = m_MapColImage.clone();
    }


    void updateMapSection() // Update the map section based on the robot's pose
    {
        // Ensure that both map_image_ and robot pose are available
        if (map_image_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Map image is not yet available.");
            return;
        }

        // Define ROI size in pixels
        double roi_width = 70;
        double roi_height = 100;

        // Calculate the robot's position in map coordinates
        double map_robot_x = (robot_x_ - origin_x) / map_scale_;
        double map_robot_y = (robot_y_ - origin_y) / map_scale_;

        // Calculate the top-left corner of the ROI
        double roi_x = map_robot_x - roi_width / 2.0;
        double roi_y = map_robot_y - roi_height / 2.0;

        // Ensure the ROI coordinates are within the bounds of the image
        roi_x = std::max(0.0, std::min(roi_x, static_cast<double>(map_image_.cols - roi_width)));
        roi_y = std::max(0.0, std::min(roi_y, static_cast<double>(map_image_.rows - roi_height)));

        // Define the ROI rectangle
        cv::Rect roi_rect(static_cast<int>(roi_x), static_cast<int>(roi_y), static_cast<int>(roi_width), static_cast<int>(roi_height));

        // Extract the ROI from the map image
        cv::Mat map_section = map_image_(roi_rect).clone();

        // Rotate the ROI based on the robot's orientation
        cv::Mat map_section_rotated;
        cv::Point2f center(map_section.cols / 2.0f, map_section.rows / 2.0f);
        double angle = robot_theta_ * 180.0 / CV_PI;

        // Get the rotation matrix
        cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, angle, 1.0);
        cv::warpAffine(map_section, map_section_rotated, rotation_matrix, map_section.size());

        // Do not plot any additional points here (comment out unwanted point plotting code)

        // Optionally, resize for better visualization
        cv::Mat map_section_resized;
        cv::resize(map_section_rotated, map_section_resized, cv::Size(), 1, 1, cv::INTER_LINEAR);

        // Create a canvas with a black background
        int canvas_size_x = map_section_resized.cols + 100;
        int canvas_size_y = map_section_resized.rows + 100;
        cv::Mat canvas = cv::Mat::zeros(canvas_size_y, canvas_size_x, CV_8UC3);

        // Place the resized map section in the center of the canvas
        int offset_x = (canvas_size_x - map_section_resized.cols) / 2;
        int offset_y = (canvas_size_y - map_section_resized.rows) / 2;
        map_section_resized.copyTo(canvas(cv::Rect(offset_x, offset_y, map_section_resized.cols, map_section_resized.rows)));

        // Update the map_section_ member variable
        map_section_ = canvas.clone();

        // Display the updated map section
        cv::imshow(WINDOW2, map_section_);
        cv::waitKey(1);
    }

    // Callback function to calculate yaw change based on feature matching
    void calculateYawChange()
    {
        std::vector<cv::Point2f> mapPoints, scanPoints;
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        std::vector<cv::DMatch> goodMatches;

        // Detect and match features between map section and scan image
        detectAndMatchFeatures(map_section_, scan_image_, mapPoints, scanPoints, keypoints1, keypoints2, goodMatches);

        if (mapPoints.size() < 3 || scanPoints.size() < 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try
        {
            // Estimate affine transformation matrix
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(mapPoints, scanPoints); 

            if (transform_matrix.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            }
            else
            {
                // Extract the rotation angle from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = angle_difference_ * 180.0 / CV_PI; // Convert to degrees
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);

                // Print the robot's relative orientation compared to the map
                double relative_orientation = robot_theta_ * 180.0 / CV_PI + angle_difference_; // Convert to degrees
                RCLCPP_INFO(this->get_logger(), "Robot's relative orientation: %f degrees", relative_orientation);
            }
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }

    // Function to filter matches using homography (optional enhancement)
    void filterMatchesByHomography(const std::vector<cv::KeyPoint> &keypoints1, const std::vector<cv::KeyPoint> &keypoints2,
                                   std::vector<cv::DMatch> &matches, std::vector<cv::DMatch> &filtered_matches)
    {
        if (matches.size() < 4)
            return; // Need at least 4 matches to compute homography

        // Convert keypoints to Point2f
        std::vector<cv::Point2f> points1, points2;
        for (const auto &match : matches)
        {
            points1.push_back(keypoints1[match.queryIdx].pt);
            points2.push_back(keypoints2[match.trainIdx].pt);
        }

        // Use RANSAC to find homography and remove outliers
        std::vector<uchar> inliersMask(points1.size());
        cv::Mat homography = cv::findHomography(points1, points2, cv::RANSAC, 3.0, inliersMask);
        // cv::Mat affine = cv::estimateAffinePartial2D(points1, points2, inliersMask);

        for (size_t i = 0; i < inliersMask.size(); i++)
        {
            if (inliersMask[i])
            {
                filtered_matches.push_back(matches[i]);
            }
        }
    }

    void detectAndMatchFeatures(const cv::Mat &img1, const cv::Mat &img2,
                                std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints,
                                std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2,
                                std::vector<cv::DMatch> &goodMatches)
    {
        // Create ORB detector with more keypoints
        cv::Ptr<cv::ORB> orb = cv::ORB::create(2000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20); // Increased keypoints
        cv::Mat descriptors1, descriptors2;

        // Detect and compute descriptors
        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        // Check if descriptors are empty
        if (descriptors1.empty() || descriptors2.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Empty descriptors, skipping feature matching.");
            return;
        }

        // Perform KNN matching
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(descriptors1, descriptors2, knn_matches, 2);

        // Apply Lowe's ratio test with slightly relaxed threshold
        const float ratio_thresh = 0.7f; // Balanced ratio for better matching
        std::set<int> matchedKeypoints1;
        std::set<int> matchedKeypoints2;

        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                int queryIdx = knn_matches[i][0].queryIdx;
                int trainIdx = knn_matches[i][0].trainIdx;

                // Check if keypoints are already matched (preventing one-to-many matches)
                if (matchedKeypoints1.find(queryIdx) == matchedKeypoints1.end() &&
                    matchedKeypoints2.find(trainIdx) == matchedKeypoints2.end())
                {

                    // Add the valid match
                    goodMatches.push_back(knn_matches[i][0]);
                    srcPoints.push_back(keypoints1[queryIdx].pt);
                    dstPoints.push_back(keypoints2[trainIdx].pt);

                    // Mark these keypoints as matched
                    matchedKeypoints1.insert(queryIdx);
                    matchedKeypoints2.insert(trainIdx);
                }
            }
        }

        // Post-process matches by removing too distant ones
        filterMatchesByHomography(keypoints1, keypoints2, goodMatches, goodMatches); // filter out outlier points

        double maxDist = 100.0; // Adjust distance threshold based on results
        for (auto it = goodMatches.begin(); it != goodMatches.end();)
        {
            cv::Point2f p1 = keypoints1[it->queryIdx].pt;
            cv::Point2f p2 = keypoints2[it->trainIdx].pt;
            double dist = cv::norm(p1 - p2);

            if (dist > maxDist || dist < 10.0)
            { // Filter out matches with a large or too small distance
                it = goodMatches.erase(it);
            }
            else
            {
                ++it;
            }
        }

        // Print out the number of valid matches found
        RCLCPP_INFO(this->get_logger(), "Good matches found: %ld", goodMatches.size());
    }

    // Function to visualize feature matches between two images
    void visualizeMatches(const cv::Mat &img1, const cv::Mat &img2)
    {
        // Detect and match features between Image B and Image C
        std::vector<cv::Point2f> mapPoints, scanPoints;
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        std::vector<cv::DMatch> goodMatches;
        detectAndMatchFeatures(img1, img2, mapPoints, scanPoints, keypoints1, keypoints2, goodMatches);
        filterMatchesByHomography(keypoints1, keypoints2, goodMatches, goodMatches);
        // Draw matches
        cv::Mat img_matches;
        cv::drawMatches(img1, keypoints1, img2, keypoints2, goodMatches, img_matches,
                        cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        // Display the matches
        cv::imshow(WINDOW4, img_matches);
        cv::waitKey(1);
    }

// ======================== Class Members ======================== //
    // ROS Subscribers and Publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    // OpenCV Images
    cv::Mat scan_image_;
    cv::Mat map_image_;
    cv::Mat map_section_;

    // Temporary Images for Map Processing
    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;

    // Robot Pose
    double robot_x_;
    double robot_y_;
    double robot_theta_; // In radians

    // Map Parameters
    double map_scale_;
    double origin_x;
    double origin_y;
    unsigned int size_x;
    unsigned int size_y;

    // Transformation Parameters
    double angle_difference_;
    double relative_orientaion_ = 0.0;

    // Window Names
    const std::string WINDOW1 = "Image: Map";
    const std::string WINDOW2 = "Image: Map section";
    const std::string WINDOW3 = "Image: Laser Scan";
    const std::string WINDOW4 = "Image: Feature detection";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
