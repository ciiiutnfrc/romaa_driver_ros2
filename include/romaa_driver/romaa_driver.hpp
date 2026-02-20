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

// Service messages
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "romaa_driver_interfaces/srv/set_odometry.hpp"

// Parameter callback return message.
#include "rcl_interfaces/msg/set_parameters_result.hpp"

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
        bool enable_motor, reset_odom;
        float wheelbase, wheel_radius;
        float v_pid_kp, v_pid_ki, v_pid_kd;
        float w_pid_kp, w_pid_ki, w_pid_kd;

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

        // Service servers
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_odom_srv;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motor_srv;
        rclcpp::Service<romaa_driver_interfaces::srv::SetOdometry>::SharedPtr set_odom_srv;

        // Service server callbacks
        void resetSrvCb(const std::shared_ptr<std_srvs::srv::Empty::Request> ,
            std::shared_ptr<std_srvs::srv::Empty::Response> );
        void resetOdometrySrvCb(const std::shared_ptr<std_srvs::srv::Empty::Request> ,
            std::shared_ptr<std_srvs::srv::Empty::Response> );
        void enableMotorSrvCb(const std::shared_ptr<std_srvs::srv::SetBool::Request> ,
            std::shared_ptr<std_srvs::srv::SetBool::Response> );
        void setOdometrySrvCb(const std::shared_ptr<romaa_driver_interfaces::srv::SetOdometry::Request> ,
            std::shared_ptr<romaa_driver_interfaces::srv::SetOdometry::Response> );

        // Parameter callback.
        OnSetParametersCallbackHandle::SharedPtr param_cb_handle;
        rcl_interfaces::msg::SetParametersResult parametersCb(
            const std::vector<rclcpp::Parameter> & );

};

} // namespace 'romaa_driver'

#endif // ROMAA_DRIVER__ROMAA_DRIVER_HPP
