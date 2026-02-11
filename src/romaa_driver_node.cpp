#include "rclcpp/rclcpp.hpp"
#include "romaa_driver/romaa_driver.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto romaa_drv = std::make_shared<romaa_driver::RoMAADriver>();

    if(rclcpp::ok())
        rclcpp::spin(romaa_drv);

    rclcpp::shutdown();
    return 0;
}
