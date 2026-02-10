#ifndef ROMAA_DRIVER__ROMAA_DRIVER_HPP
#define ROMAA_DRIVER__ROMAA_DRIVER_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"

// Messages
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

// TF2 library
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

// RoMAA communication class
#include "romaa_comm/romaa_comm.h"

namespace romaa_driver
{

class RoMAADriver : public rclcpp::Node
{
    public:
        RoMAADriver();
        ~RoMAADriver();

    private:
        double frequency;
        std::string port;                   // Serial port name
        int baudrate;                       // Serial port baudrate
        std::string odom_frame, base_frame; // TF frame names

        // Node variables
        romaa_comm *comm;
        float x, y, a, v, w;
        tf2::Quaternion quat_tf;

        // Publisher
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

        // Subscribers
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
        void cmdVelCb(geometry_msgs::msg::Twist::UniquePtr );

        // TF2 variables
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        geometry_msgs::msg::TransformStamped odom_tf;

        // Messages
        nav_msgs::msg::Odometry odom_msg;

        // Timer
        rclcpp::TimerBase::SharedPtr pub_timer;
        void pubOdometryTFCb();
};

} // namespace 'romaa_driver'

#endif // ROMAA_DRIVER__ROMAA_DRIVER_HPP
