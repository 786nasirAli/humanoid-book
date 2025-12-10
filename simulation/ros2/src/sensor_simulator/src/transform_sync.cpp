// transform_sync.cpp
// Transform synchronization between Gazebo and Unity

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

class TransformSync : public rclcpp::Node
{
public:
    TransformSync() : Node("transform_sync_node")
    {
        // Create transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // Subscribe to Gazebo odom and joint states
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&TransformSync::odom_callback, this, std::placeholders::_1));
            
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TransformSync::joint_callback, this, std::placeholders::_1));
            
        // Publisher for Unity synchronization
        unity_sync_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/unity_sync/joint_states", 10);
            
        RCLCPP_INFO(this->get_logger(), "Transform Synchronization Node Started");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Create and broadcast transform from odom to base_link
        geometry_msgs::msg::TransformStamped t;
        
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        
        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;
        
        t.transform.rotation = msg->pose.pose.orientation;
        
        tf_broadcaster_->sendTransform(t);
        
        // Publish to Unity sync topic
        publish_to_unity();
    }
    
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Update internal joint state
        joint_state_ = *msg;
        
        // Publish to Unity sync topic
        publish_to_unity();
    }
    
    void publish_to_unity()
    {
        if (unity_sync_pub_->get_subscription_count() > 0) {
            unity_sync_pub_->publish(joint_state_);
        }
    }
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr unity_sync_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    sensor_msgs::msg::JointState joint_state_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformSync>());
    rclcpp::shutdown();
    return 0;
}