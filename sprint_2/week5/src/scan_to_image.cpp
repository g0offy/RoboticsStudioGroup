#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ScanToImageNode : public rclcpp::Node
{
public:
    ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientaion_(0.0)
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(                // Create a subscriber to the /scan topic
            "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1)); // Bind the callback function
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);       // Create a publisher to the /cmd_vel topic
        RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");                           // Log a message indicating the node has started
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Purpose: This function is a callback that gets triggered whenever a new LaserScan message is received from the /scan topic.

        // 1. Convert the LaserScan message to an image
        cv::Mat image = laserScanToMat(msg); // Convert the LaserScan message to an image

        // 2. Handle the first image
        if (!first_image_captured_) // Check if the first image has been captured
        {
            original_image_ = image.clone();                          // Capture the original image
            first_image_ = image.clone();                             // Capture the first image
            first_image_captured_ = true;                             // Set the flag to indicate the first image has been captured
            cv::imshow("Original Image", original_image_);            // Display the original image
            cv::imshow("First Image", first_image_);                  // Display the first image
            cv::waitKey(1);                                           // Add this to process GUI events and update the window
            RCLCPP_INFO(this->get_logger(), "First image captured."); // Log a message indicating the first image has been captured
            MoveRobot();
        }
        else if (!second_image_captured_) // Capture the second image
        {
            second_image_ = image.clone();
            second_image_captured_ = true;                             // Set the flag to indicate the second image has been captured
            cv::imshow("Second Image", second_image_);                 // Display the second image
            cv::waitKey(1);                                            // Add this to process GUI events and update the window
            RCLCPP_INFO(this->get_logger(), "Second image captured."); // Log a message indicating the second image has been captured
            MoveRobot();
        }
        else
        {
            first_image_ = second_image_.clone();
            second_image_ = image.clone();
            // Display the new second image
            cv::imshow("Second Image", second_image_);
            cv::waitKey(1); // Add this to process GUI events and update the window

            calculateYawChange(first_image_, second_image_);
            relative_orientaion_ = relative_orientaion_ + angle_difference_;
            RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f", relative_orientaion_);
            MoveRobot();
        }
    }

    void MoveRobot()
    {
        // Purpose: This function moves the robot to the right by publishing a Twist message with a positive linear velocity in the y-direction.

        // 1. Create a Twist message to send velocity commands
        geometry_msgs::msg::Twist cmd_msg; // Create a Twist message to send velocity commands

        // 2. Set the linear velocity in the y-direction to move right
        cmd_msg.angular.z = 1.0;                     // Set a positive value for linear velocity in the y-direction to move right
        cmd_publisher_->publish(cmd_msg);           // Publish the command
        rclcpp::sleep_for(std::chrono::seconds(2)); // Sleep for a while to allow the robot to move

        // 3. Stop the robot by setting all velocities to zero
        cmd_msg.linear.x = 0.0;
        cmd_msg.linear.y = 0.0;
        cmd_msg.linear.z = 0.0;
        cmd_msg.angular.x = 0.0;
        cmd_msg.angular.y = 0.0;
        cmd_msg.angular.z = 0.0;
        cmd_publisher_->publish(cmd_msg); // Publish the stop command
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {

        // Purpose: Converts a LaserScan message into a binary image (cv::Mat), where each pixel represents the presence of an obstacle detected by the laser scanner.

        // Functionality:

        //      Create Image: Initializes a blank image of size 500x500 pixels.
        //      Map Polar to Cartesian: Iterates over the laser scan data, converting polar coordinates (distance and angle) to Cartesian coordinates (x, y) and sets the corresponding pixel in the image to white (255) if within range.

        // 1. Initialize a blank image of size 500x500 pixels
        cv::Mat image = cv::Mat::zeros(500, 500, CV_8UC1);

        // 2. Iterate over the laser scan data and convert polar to Cartesian coordinates
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {

            double angle = scan->angle_min + i * scan->angle_increment; // Calculate the angle
            double x = scan->ranges[i] * std::cos(angle);               // Calculate the x-coordinate
            double y = scan->ranges[i] * std::sin(angle);               // Calculate the y-coordinate

            // Map the Cartesian coordinates to image pixels
            int pixel_x = static_cast<int>((x + 10.0) * 25.0); // Scale and shift x
            int pixel_y = static_cast<int>((y + 10.0) * 25.0); // Scale and shift y

            // Set the pixel value to 255 (white) if within range
            if (pixel_x >= 0 && pixel_x < 500 && pixel_y >= 0 && pixel_y < 500)
            {                                            // Check if the pixel is within the image bounds
                image.at<uchar>(pixel_y, pixel_x) = 255; // Set the pixel value to 255 (white)
            }
        }

        // 3. Return the binary image
        return image;
    }

    double calculateYawChange(const cv::Mat &img1, const cv::Mat &img2)
    {
        // Purpose: Estimates the change in orientation (yaw angle) of the robot by comparing two images.

        // Functionality:

        //     Feature Matching: Uses feature detection and matching to find corresponding points between the two images.
        // std::vector<cv::Point2f> srcPoints, dstPoints;
        // detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);
        //     Estimate Transformation: Computes an affine transformation matrix to determine the rotation between the two images.
        //     Calculate Angle: Extracts the rotation angle from the transformation matrix and converts it to degrees.

        // 1. Detect and match features between the two images
        std::vector<cv::Point2f> srcPoints, dstPoints;            // Initialize vectors to store source and destination points
        detectAndMatchFeatures(img1, img2, srcPoints, dstPoints); // Detect and match features

        // 2. Compute the affine transformation matrix
        cv::Mat transformation = cv::estimateAffinePartial2D(srcPoints, dstPoints); // Estimate the affine transformation matrix

        // 3. Extract the rotation angle from the transformation matrix
        double angle = std::atan2(transformation.at<double>(1, 0), transformation.at<double>(0, 0)); // Calculate the rotation angle
        double angle_deg = angle * 180.0 / CV_PI;                                                    // Convert the angle to degrees

        // 4. Update the relative orientation
        relative_orientaion_ += angle_deg; // Update the relative orientation

        // 5. Return the rotation angle in degrees
        return angle_deg; // Return the rotation angle in degrees
    }

    void detectAndMatchFeatures(const cv::Mat &img1, const cv::Mat &img2,
                                std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints)
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b)
                  { return a.distance < b.distance; });

        // Determine the number of top matches to keep (30% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto &match : matches)
        {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    cv::Mat first_image_, second_image_, original_image_;
    bool first_image_captured_ = false;
    bool second_image_captured_ = false;

    double angle_difference_;
    double relative_orientaion_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToImageNode>());
    rclcpp::shutdown();
    return 0;
}