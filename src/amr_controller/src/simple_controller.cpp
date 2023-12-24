
#include <Eigen/Geometry>
#include "amr_controller/simple_controller.hpp"

using std::placeholders::_1;


SimpleController::SimpleController(const std::string& name)
                                    : Node(name)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation_lr", 0.17);
    declare_parameter("wheel_separation_fr", 0.17);
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_lr_ = get_parameter("wheel_separation_lr").as_double();
    wheel_separation_fr_ = get_parameter("wheel_separation_fr").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation lr" << wheel_separation_lr_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation fr" << wheel_separation_fr_);
    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/amr_controller/cmd_vel", 10, std::bind(&SimpleController::velCallback, this, _1));

speed_conversion_ << wheel_radius_/2, wheel_radius_/2, wheel_radius_/2, wheel_radius_/2,
                    wheel_radius_/wheel_separation_lr_, -wheel_radius_/wheel_separation_lr_, wheel_radius_/wheel_separation_fr_, -wheel_radius_/wheel_separation_fr_;    
                    RCLCPP_INFO_STREAM(get_logger(), "The conversion matrix is \n" << speed_conversion_);
}


void SimpleController::velCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    // Implements the differential kinematic model
    // Given v and w, calculate the velocities of the wheels
    Eigen::Vector4d robot_speed(msg.twist.linear.x, msg.twist.angular.z, 0.0, 0.0);
    Eigen::Vector4d wheel_speed = speed_conversion_.inverse() * robot_speed;
    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(2));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(3));

    wheel_cmd_pub_->publish(wheel_speed_msg);
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}