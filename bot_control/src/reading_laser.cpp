#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <algorithm> 

class LaserScanFilter : public rclcpp::Node {
public:
    LaserScanFilter() : Node("laser_scan_filter") {
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, 
            std::bind(&LaserScanFilter::scanCallback, this, std::placeholders::_1)
        );

        filtered_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/filtered_scan", 10
        );
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr original_scan) {
        auto filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        
        *filtered_scan = *original_scan;

        double min_angle = 0.0;  // 0 degrees
        double max_angle = M_PI * 1/3;  // 120 degrees

        size_t start_index = static_cast<size_t>(std::max(0.0, 
            std::floor((min_angle - original_scan->angle_min) / original_scan->angle_increment)
        ));
        
        size_t end_index = static_cast<size_t>(std::min(
            static_cast<double>(original_scan->ranges.size()),
            std::ceil((max_angle - original_scan->angle_min) / original_scan->angle_increment)
        ));

        filtered_scan->ranges.clear();
        filtered_scan->intensities.clear();

        for (size_t i = start_index; i < end_index && i < original_scan->ranges.size(); ++i) {
            filtered_scan->ranges.push_back(original_scan->ranges[i]);
            
            if (!original_scan->intensities.empty() && i < original_scan->intensities.size()) {
                filtered_scan->intensities.push_back(original_scan->intensities[i]);
            }
        }

        filtered_scan->angle_min = min_angle;
        filtered_scan->angle_max = max_angle;
        filtered_scan->angle_increment = original_scan->angle_increment;

        filtered_scan->scan_time = original_scan->scan_time;
        filtered_scan->time_increment = original_scan->time_increment;
        filtered_scan->header = original_scan->header;

        // Publish filtered scan
        filtered_scan_publisher_->publish(*filtered_scan);

        RCLCPP_INFO(this->get_logger(), "Filtered scan published with %zu ranges", 
                    filtered_scan->ranges.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
