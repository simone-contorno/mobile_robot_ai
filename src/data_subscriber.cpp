#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std;

class DataSubscriber : public rclcpp::Node
{
public:
    // Public constructor names the node and initialize count_ to 0
    DataSubscriber() : Node("data_subscriber")
    {
        // Create the subscriber (name, queue size, callback function)
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&DataSubscriber::scan_callback, this, std::placeholders::_1));
    }

private:
    // Create the private subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    
    // Create the callback function
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "New scan data: %f", msg->ranges[0]);
    }
};

int main(int argc, char* argv[])
{
    // Initialize the node
    rclcpp::init(argc, argv);
    
    // Start the callback function
    rclcpp::spin(std::make_shared<DataSubscriber>());

    // Terminate the node
    rclcpp::shutdown();

    return 0;
}