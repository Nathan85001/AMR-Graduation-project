#include "amr_controller/simple_controller.hpp"
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

using std::placeholders::_1;



using std::placeholders::_1;


SimpleController::SimpleController(const std::string& name)
                                : Node(name)
                                , front_left_wheel_prev_pos_(0.0)
                                , front_right_wheel_prev_pos_(0.0)
                                , rear_left_wheel_prev_pos_(0.0) 
                                , rear_right_wheel_prev_pos_(0.0)   
                                , x_(0.0)
                                , y_(0.0)
                                , theta_(0.0)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation " << wheel_separation_);
    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/amr_controller/cmd_vel", 10, std::bind(&SimpleController::velCallback, this, _1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&SimpleController::jointCallback, this, _1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/amr_controller/odom", 10);

speed_conversion_ <<
    wheel_radius_/4,  wheel_radius_/4, wheel_radius_/4, wheel_radius_/4,
    -wheel_radius_/(4*wheel_separation_),  -wheel_radius_/(4*wheel_separation_), wheel_radius_/(4*wheel_separation_), wheel_radius_/(4*wheel_separation_),
    0,  0,  0,  0,
    0,  0,  0,  0;
    RCLCPP_INFO_STREAM(get_logger(), "The conversion matrix is \n" << speed_conversion_);

    // Fill the Odometry message with invariant parameters
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint";

    prev_time_ = get_clock()->now();
}


void SimpleController::velCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    // Implements the 4WD differential kinematic model
    // Given v and w, calculate the velocities of the four wheels
    Eigen::Vector4d robot_speed(msg.twist.linear.x, 0, msg.twist.angular.z, 0); // Note the addition of 0 for linear Y
    Eigen::Vector4d wheel_speed = speed_conversion_.inverse() * robot_speed;

    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed(0));
    wheel_speed_msg.data.push_back(wheel_speed(1));
    wheel_speed_msg.data.push_back(wheel_speed(2));
    wheel_speed_msg.data.push_back(wheel_speed(3));

    wheel_cmd_pub_->publish(wheel_speed_msg);
}

void SimpleController::jointCallback(const sensor_msgs::msg::JointState &state)
{
    // Implements the inverse differential kinematic model
    // Given the position of the wheels, calculates their velocities
    // then calculates the velocity of the robot wrt the robot frame
    // and then converts it in the global frame and publishes the TF
    double dp_front_left = state.position.at(1) - front_left_wheel_prev_pos_;
    double dp_front_right = state.position.at(0) - front_right_wheel_prev_pos_;
    double dp_rear_left = state.position.at(3) - rear_left_wheel_prev_pos_;
    double dp_rear_right = state.position.at(2) - rear_right_wheel_prev_pos_;
    rclcpp::Time msg_time = state.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    // Actualize the prev pose for the next itheration
    front_left_wheel_prev_pos_ = state.position.at(1);
    front_right_wheel_prev_pos_ = state.position.at(0);
    rear_left_wheel_prev_pos_ = state.position.at(3);
    rear_right_wheel_prev_pos_ = state.position.at(2);
    prev_time_ = state.header.stamp;

    // Calculate the rotational speed of each wheel
    double front_fi_left = dp_front_left / dt.seconds();
    double front_fi_right = dp_front_right / dt.seconds();
    double rear_fi_left = dp_rear_left / dt.seconds();
    double rear_fi_right = dp_rear_right / dt.seconds();

    // Calculate the linear and angular velocity
    double front_linear = (wheel_radius_ * front_fi_right + wheel_radius_ * front_fi_left) / 2;
    double rear_linear = (wheel_radius_ * rear_fi_right + wheel_radius_ * rear_fi_left) / 2;
    // double front_angular = abs((wheel_radius_ * front_fi_right - wheel_radius_ * front_fi_left)) / wheel_separation_;
    // double rear_angular = abs((wheel_radius_ * rear_fi_right - wheel_radius_ * rear_fi_left)) / wheel_separation_;



    // Calculate the position increment
    double front_d_s = (wheel_radius_ * dp_front_right + wheel_radius_ * dp_front_left) / 2;
    double rear_d_s = (wheel_radius_ * dp_rear_right + wheel_radius_ * dp_rear_left) / 2;
    double avg_d_s = (front_d_s + rear_d_s) / 2;

    double front_d_theta = (wheel_radius_ * dp_front_right - wheel_radius_ * dp_front_left) / wheel_separation_;
    double rear_d_theta = (wheel_radius_ * dp_rear_right - wheel_radius_ * dp_rear_left) / wheel_separation_;
    double avg_d_theta = (front_d_theta + rear_d_theta) / 2;

    theta_ += avg_d_theta;
    double linearx =  (front_linear * sin(abs(theta_)))- (rear_linear * sin(abs(theta_)));
    double lineary =  (front_linear * cos(abs(theta_)));
    double angular = (abs(front_linear*sin(abs(theta_)))+abs(rear_linear*sin(abs(theta_) )))/wheel_separation_;
    x_ += avg_d_s * cos(theta_);
    y_ += avg_d_s * sin(theta_);

    // Compose and publish the odom message
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg_.header.stamp = get_clock()->now();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.orientation.x = q.getX();
    odom_msg_.pose.pose.orientation.y = q.getY();
    odom_msg_.pose.pose.orientation.z = q.getZ();
    odom_msg_.pose.pose.orientation.w = q.getW();
    odom_msg_.twist.twist.linear.x = linearx;
    odom_msg_.twist.twist.linear.y = lineary;
    odom_msg_.twist.twist.angular.z = angular;
    odom_pub_->publish(odom_msg_);

    // TF
    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.getX();
    transform_stamped_.transform.rotation.y = q.getY();
    transform_stamped_.transform.rotation.z = q.getZ();
    transform_stamped_.transform.rotation.w = q.getW();
    transform_stamped_.header.stamp = get_clock()->now();
    transform_broadcaster_->sendTransform(transform_stamped_);
}



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}