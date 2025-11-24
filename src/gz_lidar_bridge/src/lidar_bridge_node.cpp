#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/transport/Node.hh>

class LidarBridgeNode : public rclcpp::Node
{
public:
    LidarBridgeNode() : Node("lidar_bridge_node")
    {
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
        
        if (!gz_node_.Subscribe("/scan", &LidarBridgeNode::OnLidarMsg, this))
        {
            RCLCPP_ERROR(this->get_logger(), "Error subscribing to Gazebo /scan");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Subscribed to Gazebo /scan");
        }
        
        if (!gz_node_.Subscribe("/clock", &LidarBridgeNode::OnClockMsg, this))
        {
            RCLCPP_ERROR(this->get_logger(), "Error subscribing to Gazebo /clock");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Subscribed to Gazebo /clock");
        }
    }

private:
    void OnLidarMsg(const gz::msgs::LaserScan &gz_msg)
    {
        sensor_msgs::msg::LaserScan ros_msg;
        
        ros_msg.header.stamp = this->now();
        ros_msg.header.frame_id = "base_link";
        
        ros_msg.angle_min = gz_msg.angle_min();
        ros_msg.angle_max = gz_msg.angle_max();
        ros_msg.angle_increment = gz_msg.angle_step();
        ros_msg.time_increment = 0.0;
        ros_msg.scan_time = 0.0;
        ros_msg.range_min = gz_msg.range_min();
        ros_msg.range_max = gz_msg.range_max();
        
        ros_msg.ranges.resize(gz_msg.ranges_size());
        for (int i = 0; i < gz_msg.ranges_size(); ++i)
        {
            ros_msg.ranges[i] = gz_msg.ranges(i);
        }
        
        if (gz_msg.intensities_size() > 0)
        {
            ros_msg.intensities.resize(gz_msg.intensities_size());
            for (int i = 0; i < gz_msg.intensities_size(); ++i)
            {
                ros_msg.intensities[i] = gz_msg.intensities(i);
            }
        }
        
        scan_pub_->publish(ros_msg);
    }
    
    void OnClockMsg(const gz::msgs::Clock &gz_msg)
    {
        rosgraph_msgs::msg::Clock ros_msg;
        
        // Convert Gazebo time to ROS time
        ros_msg.clock.sec = gz_msg.sim().sec();
        ros_msg.clock.nanosec = gz_msg.sim().nsec();
        
        clock_pub_->publish(ros_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    gz::transport::Node gz_node_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
