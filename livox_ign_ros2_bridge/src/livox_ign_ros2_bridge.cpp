#include <rclcpp/rclcpp.hpp>
#include <ignition/transport/Node.hh>
#include "CustomMsg.pb.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>

class LivoxIgnRos2Bridge : public rclcpp::Node
{
public:
    LivoxIgnRos2Bridge() : Node("livox_ign_ros2_bridge")
    {
        // parameter
        std::string ign_topic = this->declare_parameter<std::string>("ign_topic", "/livox_points");
        std::string ros_topic = this->declare_parameter<std::string>("ros_topic", "livox_points");
        
        // Ignition Subscriber
        _ign_node.Subscribe(ign_topic, &LivoxIgnRos2Bridge::callback_ign, this);

        // ROS2 Publisher
        _pub = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(ros_topic, 10);
    }

private:
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr _pub;
    ignition::transport::Node _ign_node;

    void callback_ign(const LivoxCustomMsg::CustomMsg &msg)
    {
    	auto ros_msg = livox_ros_driver2::msg::CustomMsg();
    	
        ros_msg.header.stamp = this->get_clock()->now();
        //ros_msg.header.stamp.sec     = msg.stamp().sec();
        //ros_msg.header.stamp.nanosec = msg.stamp().nsec();
        ros_msg.header.frame_id      = msg.frame_id();

        ros_msg.timebase  = msg.time_base();
        ros_msg.point_num = msg.point_num();
        ros_msg.lidar_id  = msg.lidar_id();

        for ( uint32_t i = 0; i < (uint32_t)msg.points_size(); i++) {
            const auto &point = msg.points(i);

            livox_ros_driver2::msg::CustomPoint ros_point;
            ros_point.offset_time  = point.offset_time();
            ros_point.x            = point.x();
            ros_point.y            = point.y();
            ros_point.z            = point.z();
            ros_point.reflectivity = point.reflectivity();
            ros_point.tag          = point.tag();
            ros_point.line         = point.line();

            ros_msg.points.push_back(ros_point);
        }
        
        _pub->publish(ros_msg);        
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<LivoxIgnRos2Bridge>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
    /*
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxIgnRos2Bridge>());
    rclcpp::shutdown();
    return 0;
    */
}

