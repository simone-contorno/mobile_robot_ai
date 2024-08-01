#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/path.hpp"

//#include "tf2/LinearMath/Quaternion.h"
//#include "tf2/LinearMath/Matrix3x3.h"

using namespace std;

double path_percentage = 50.0;

class DataSubscriber : public rclcpp::Node
{
public:
    // Public constructor names the node and initialize count_ to 0
    DataSubscriber() : Node("data_subscriber")
    {   
        /* Publishers (name, queue size) */

        // Laser scanner
        publisher_ranges = this->create_publisher<std_msgs::msg::Float32MultiArray>("ranges", 10);

        // Path waypoints
        publisher_plan = this->create_publisher<nav_msgs::msg::Path>("plan", 10);

        /* Subscribers (name, queue size, callback function) */

        // Laser scanner
        subscription_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&DataSubscriber::scan_callback, this, std::placeholders::_1));

        // Plan
        subscription_plan = this->create_subscription<nav_msgs::msg::Path>(
            "mobile_robot_ai/plan", 10, std::bind(&DataSubscriber::plan_callback, this, std::placeholders::_1));
    }

private:
    // Private subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_scan;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_plan;

    // Private publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_ranges;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_plan;

    // Private variables
    std::vector<float> ranges_;
    //geometry_msgs::msg::PoseStamped waypoint_;

    std_msgs::msg::Float32MultiArray msg_ranges;
    nav_msgs::msg::Path msg_plan;
    
    /* Create the callback function */

    // Laser scanner
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        ranges_.clear();
        for (int i = 0; i < msg->ranges.size(); i++)
            ranges_.push_back(msg->ranges[i]);
        
        //RCLCPP_INFO(this->get_logger(), "New scan data [%i]", msg->ranges.size());

        // Publish the data
        msg_ranges.data = ranges_;
        publisher_ranges->publish(msg_ranges);
    }

    // Plan
    void plan_callback(const nav_msgs::msg::Path::SharedPtr msg) 
    {
        if (msg->poses.size() != 0)
        {
            RCLCPP_INFO(this->get_logger(), "New plan of %i waypoints heard!", msg->poses.size());

            // Clear current plan
            msg_plan.poses.clear();
            msg_plan.header.frame_id = "map";

            // Choose a step to take equidistant waypoints
            double num_points = msg->poses.size() / 100.0 * path_percentage;
            int step = static_cast<int>(msg->poses.size() / num_points);

            // Add intermediate waypoints
            for (int i = 0; i < msg->poses.size(); i += step)
            {
                geometry_msgs::msg::PoseStamped waypoint_;
                waypoint_.header.frame_id = "map";

                waypoint_.pose.position.x = msg->poses[i].pose.position.x;
                waypoint_.pose.position.y = msg->poses[i].pose.position.y;
                
                waypoint_.pose.orientation.x = msg->poses[i].pose.orientation.x;
                waypoint_.pose.orientation.y = msg->poses[i].pose.orientation.y;
                waypoint_.pose.orientation.z = msg->poses[i].pose.orientation.z;
                waypoint_.pose.orientation.w = msg->poses[i].pose.orientation.w;            
                
                msg_plan.poses.push_back(waypoint_);
            }

            // Add last waypoint
            geometry_msgs::msg::PoseStamped waypoint_;
            waypoint_.header.frame_id = "map";

            waypoint_.pose.position.x = msg->poses[msg->poses.size() - 1].pose.position.x;
            waypoint_.pose.position.y = msg->poses[msg->poses.size() - 1].pose.position.y;
            
            waypoint_.pose.orientation.x = msg->poses[msg->poses.size() - 1].pose.orientation.x;
            waypoint_.pose.orientation.y = msg->poses[msg->poses.size() - 1].pose.orientation.y;
            waypoint_.pose.orientation.z = msg->poses[msg->poses.size() - 1].pose.orientation.z;
            waypoint_.pose.orientation.w = msg->poses[msg->poses.size() - 1].pose.orientation.w;            
            
            msg_plan.poses.push_back(waypoint_);

            // Publish the data
            publisher_plan->publish(msg_plan);

            RCLCPP_INFO(this->get_logger(), "New plan of %i waypoints published", msg_plan.poses.size());
        }
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