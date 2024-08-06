#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/path.hpp"

#include <fstream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std;

float path_percentage = 50.0;

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

        // Goal
        subscription_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&DataSubscriber::goal_callback, this, std::placeholders::_1));
    }

private:
    // Private subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_scan;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_plan;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_goal;

    // Private publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_ranges;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_plan;

    // Private variables
    std::vector<float> ranges_;

    bool new_goal = false;
    geometry_msgs::msg::Pose goal;

    std_msgs::msg::Float32MultiArray msg_ranges;
    nav_msgs::msg::Path msg_plan;
    
    /* Create the callback function */

    // Goal
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "New goal received!");
        
        if (goal != msg->pose)
        {
            new_goal = true;
            goal = msg->pose;
        }
    }
    
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
            if (new_goal == true)
            {
                new_goal = false;
                publisher_plan->publish(msg_plan);
                RCLCPP_INFO(this->get_logger(), "New plan of %i waypoints published", msg_plan.poses.size());
            }
        }
    }
};


int main(int argc, char* argv[])
{
    std::string package_share_directory;
    try {
        // Get the package share directory
        std::string package_name = "mobile_robot_ai";
        package_share_directory = ament_index_cpp::get_package_share_directory(package_name);

    } catch (const std::exception &e) {
        std::cerr << "Error locating package directory: " << e.what() << std::endl;
        return 1;
    }

    std::string file = package_share_directory + "/control_config.txt";
    std::string key = "path_percentage";

    // Open the file
    std::ifstream config_file;
    try {
        config_file.open(file, std::ios::in);
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
    }

    // Check if the file was successfully opened
    if (!config_file.is_open()) {
        std::cerr << "Error opening file: " << file << std::endl;
        
        // Check specific error conditions
        if (errno) {
            std::cerr << "Error: " << strerror(errno) << std::endl;
        }

        if (config_file.fail()) {
            std::cerr << "General failure error." << std::endl;
        }
        
        if (config_file.bad()) {
            std::cerr << "Read/write error on i/o operation." << std::endl;
        }

        if (config_file.eof()) {
            std::cerr << "End-of-File reached on input operation." << std::endl;
        }
        
        return 1;
    }
    
    // Read the file line by line
    std::string line;
    while (getline(config_file, line)) {

        // Check if the line contains the key
        size_t pos = line.find(key);

        if (pos != std::string::npos) {

            // Extract the value part
            size_t start = line.find('=', pos);

            if (start != std::string::npos) {

                // Move to the position after the '=' character
                start++;

                // Trim any leading whitespace
                while (start < line.size() && std::isspace(line[start])) {
                    start++;
                }
                
                // Extract the numeric value as a substring
                std::string valueStr = line.substr(start);

                // Convert the string value to float
                bool valid = true;
                try { // Attempt to convert string to float
                    std::stof(valueStr);  
                } catch (const std::invalid_argument& e) { // Conversion failed due to invalid argument
                    cout << "Path percentage invalid argument: " << e.what() << endl;
                    valid = false;
                } catch (const std::out_of_range& e) {  // Conversion failed due to out of range error
                    cout << "Path percentage out of range: " << e.what() << endl;
                    valid = false;   
                }

                if (valid == true)
                    path_percentage = std::stof(valueStr);
            }
        }
    }
    std::cout << key << ": " << path_percentage << std::endl;

    // Initialize the node
    rclcpp::init(argc, argv);
    
    // Start the callback function
    rclcpp::spin(std::make_shared<DataSubscriber>());

    // Terminate the node
    rclcpp::shutdown();

    return 0;
}