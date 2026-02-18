#include "rclcpp/rclcpp.hpp"
#include "romaa_driver/romaa_driver.hpp"

namespace romaa_driver
{

using std::placeholders::_1;
using std::placeholders::_2;

RoMAADriver::RoMAADriver() : Node("romaa_driver")
{
    // Declare node paremeters
    declare_parameter<double>("frequency", 10.0);
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 115200);
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<bool>("enable_motor", false);
    declare_parameter<bool>("reset_odom", false);

    // Reading parameters
    frequency = get_parameter("frequency").as_double();
    port = get_parameter("port").as_string();
    baudrate = get_parameter("baudrate").as_int();
    odom_frame = get_parameter("odom_frame").as_string();
    base_frame = get_parameter("base_frame").as_string();
    enable_motor = get_parameter("enable_motor").as_bool();
    reset_odom = get_parameter("reset_odom").as_bool();

    RCLCPP_INFO(get_logger(), "Driver node parameters ready.");

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

    // Enable/disable motors
    if(enable_motor == true)
    {
        comm->enable_motor();
        RCLCPP_INFO(get_logger(), "Enable motors.");
    }
    else
    {
        comm->disable_motor();
        RCLCPP_INFO(get_logger(), "Disable motors.");
    }

    // Reset odometry
    if(reset_odom == true)
    {
        comm->reset_odometry();
        RCLCPP_INFO(get_logger(), "Reset odometry.");
    }

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

    // Service servers
    reset_srv = create_service<std_srvs::srv::Empty>("reset",
        std::bind(&RoMAADriver::resetSrvCb, this, _1, _2));
    reset_odom_srv = create_service<std_srvs::srv::Empty>("reset_odometry",
        std::bind(&RoMAADriver::resetOdometrySrvCb, this, _1, _2));
    motor_srv = create_service<std_srvs::srv::SetBool>("enable_motor",
        std::bind(&RoMAADriver::enableMotorSrvCb, this, _1, _2));

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

void RoMAADriver::cmdVelCb(geometry_msgs::msg::Twist::UniquePtr msg)
{
    RCLCPP_INFO(get_logger(), "v: %.2f, w: %.2f", msg->linear.x, msg->angular.z);
    comm->set_speed(msg->linear.x, msg->angular.z);
}

void RoMAADriver::pubOdometryTFCb()
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

// Service callback
void RoMAADriver::resetSrvCb(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request;
    (void)response;
    RCLCPP_INFO(get_logger(), "Reset embedded controller.");
    comm->reset();
}

// Service callback
void RoMAADriver::resetOdometrySrvCb(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request;
    (void)response;
    RCLCPP_INFO(get_logger(), "Reset odometry.");
    comm->reset_odometry();
}

// Service callback
void RoMAADriver::enableMotorSrvCb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if(request->data == true)
    {
        RCLCPP_INFO(get_logger(), "Enable motor.");
        comm->enable_motor();
        response->success = true;
        response->message = "Motor enabled.";
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Disable motor.");
        comm->disable_motor();
        response->success = true;
        response->message = "Motor disabled.";
    }
}

} // namespace 'romaa_driver'
