#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;

class DataSubscriber : public rclcpp::Node
{
public:
    // Public constructor names the node and initialize count_ to 0
    DataSubscriber() : Node("data_subscriber")
    {   
        // Create the publisher (name, queue size)
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("ranges", 10);

        // Create the subscriber (name, queue size, callback function)
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&DataSubscriber::scan_callback, this, std::placeholders::_1));
    }

private:
    // Create the private subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    // Create the private publisher
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    // Create the ranges vector
    std::vector<float> ranges_;
    std_msgs::msg::Float32MultiArray msg_;
    
    // Create the callback function
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        ranges_.clear();
        for (int i = 0; i < msg->ranges.size(); i++)
            ranges_.push_back(msg->ranges[i]);
        
        RCLCPP_INFO(this->get_logger(), "New scan data [%i]", msg->ranges.size());

        // Publish the data
        msg_.data = ranges_;
        publisher_->publish(msg_);
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