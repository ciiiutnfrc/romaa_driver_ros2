#include "rclcpp/rclcpp.hpp"
#include "romaa_driver/romaa_driver.hpp"

namespace romaa_driver
{

using std::placeholders::_1;
using std::placeholders::_2;

RoMAADriver::RoMAADriver() : Node("romaa_driver")
{
    frequency = 10.0;
    port = "/dev/ttyUSB0";
    baudrate = 115200;
    odom_frame = "odom";
    base_frame = "base_link";

    // Create communication object
    RCLCPP_INFO(get_logger(), "Opening RoMAA communication port in %s at %d...",
        port.c_str(), baudrate);
    comm = new romaa_comm(port.c_str(), baudrate);

    // Check if connection is Ok
    if(comm->is_connected() == true)
        RCLCPP_INFO(get_logger(), "Connected to RoMAA.");
    else
    {
        RCLCPP_ERROR(get_logger(), "Could not connect to RoMAA.");
        rclcpp::shutdown();
        return;
    }

   comm->enable_motor();
   RCLCPP_INFO(get_logger(), "Enable motors.");

    // Create publisher
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Create subscriber
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SensorDataQoS(),
        std::bind(&RoMAADriver::cmdVelCb, this, _1));

    // Set odometry and TF messages constant fields
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_frame;
    odom_tf.header.frame_id = odom_frame;
    odom_tf.child_frame_id = base_frame;

    // Transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Publisher timer
    pub_timer = create_wall_timer(std::chrono::duration<double>(1.0 / frequency),
        std::bind(&RoMAADriver::pubOdometryTFCb, this));
}

RoMAADriver::~RoMAADriver()
{
    RCLCPP_INFO(get_logger(), "Terminating driver node.");
    if(comm->is_connected() == true)
    {
        RCLCPP_INFO(get_logger(), "Closing RoMAA communication.");
        comm->set_speed(0.0, 0.0);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        comm->disable_motor();
    }
    RCLCPP_INFO(get_logger(), "Deleting comm. object");
    delete comm;
}

void
RoMAADriver::cmdVelCb(geometry_msgs::msg::Twist::UniquePtr msg)
{
    RCLCPP_INFO(get_logger(), "v: %.2f, w: %.2f", msg->linear.x, msg->angular.z);
    comm->set_speed(msg->linear.x, msg->angular.z);
}

void
RoMAADriver::pubOdometryTFCb()
{
    // Time
    auto current_time = now();

    // Read odometry
    if( comm->get_odometry(x, y, a) == -1 )
        RCLCPP_INFO(get_logger(), "Unable to read odometry!");
    if( comm->get_speed(v, w) == -1 )
        RCLCPP_INFO(get_logger(), "Unable to read speed1");

    // Odometry message
    odom_msg.header.stamp = current_time;
    // Robot position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    // Yaw angle to quaternion
    quat_tf.setRPY(0, 0, a);
    odom_msg.pose.pose.orientation = tf2::toMsg(quat_tf);

    // Odometry twist
    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.angular.z = w;

    // TF
    odom_tf.header.stamp = current_time;
    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.rotation = odom_msg.pose.pose.orientation;

    // Publish odometry message
    odom_pub->publish(odom_msg);

    // Broadcast TF
    tf_broadcaster->sendTransform(odom_tf);
}

} // namespace 'romaa_driver'
