#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanSubscriber : public rclcpp::Node
{
public:
    LaserScanSubscriber() : Node("laser_scan_subscriber")
    {
        // Create a subscriber to the LaserScan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanSubscriber::scanCallback, this, std::placeholders::_1));
        // Create a publisher for the republished scan data
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 100);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {

        auto scan_subset = std::make_shared<sensor_msgs::msg::LaserScan>();

        // Range
        float start_angle = (360 - 30.0) * (M_PI / 180.0);
        float end_angle = 30.0 * (M_PI / 180.0);

        // find the index at which the scan is taken
        int half1_start_index = 0;
        int half1_end_index = static_cast<int>(end_angle / msg->angle_increment);
        int half2_start_index = static_cast<int>((start_angle) / msg->angle_increment);
        int half2_end_index = static_cast<int>(msg->ranges.size());

        // Ensure indices are within the valid range
        half1_end_index = std::min(half1_end_index, static_cast<int>(msg->ranges.size()) - 1);
        half2_start_index = std::max(half2_start_index, 0);
        half2_end_index = std::min(half2_end_index, static_cast<int>(msg->ranges.size()) - 1);

        // Create new vectors with the desired subsets
        std::vector<float> part2(msg->ranges.begin() + half1_start_index, msg->ranges.begin() + half1_end_index + 1);
        std::vector<float> part1(msg->ranges.begin() + half2_start_index, msg->ranges.begin() + half2_end_index + 1);

        // Combine the subsets
        std::vector<float> combined_ranges;
        combined_ranges.reserve(part1.size() + part2.size());
        combined_ranges.insert(combined_ranges.end(), part1.begin(), part1.end());
        combined_ranges.insert(combined_ranges.end(), part2.begin(), part2.end());

        scan_subset = msg; // Copy the original message

        scan_subset->ranges = std::move(combined_ranges); // std::vector<float>(msg->ranges.begin() + max_index, msg->ranges.begin() + min_index - 1);
        scan_subset->angle_min = start_angle;
        scan_subset->angle_max = end_angle;

        publisher_->publish(*scan_subset);

        // Example: Print the range of the first measurement
        RCLCPP_INFO(this->get_logger(), "half2_start_index: %d", half2_start_index);
        RCLCPP_INFO(this->get_logger(), "half2_end_index: %d", half2_end_index);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<LaserScanSubscriber>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}
