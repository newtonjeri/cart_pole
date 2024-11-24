#ifndef CART_POLE_DE_SIMULATION_NODE_HPP_
#define CART_POLE_DE_SIMULATION_NODE_HPP_

#include <algorithm>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace cart_pole_de
{
class DeSimulationNode : public rclcpp::Node
{
public:
  DeSimulationNode(const rclcpp::NodeOptions & options);

private:
  static constexpr double g_ = 9.80665;
  double dt_;

  
  // Mechanical System parameters
  double m1_, m2_;///M and m
  double l1_;

  double position_, theta_;
  double dposition_, dtheta_;
  double ddposition_ = 0.0, ddtheta_ = 0.0;

  double tau1_, tau2_;
  
  double max_torque_;
  double max_velocity_slider_1_;
  double max_velocity_continuous_revolute_1_;

  double current_time_ = 0.0;

  // Initial state
  Eigen::Vector2d q_;           // [x, theta]
  Eigen::Vector2d q_dot_;       // [x_dot, theta_dot]

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_disturbance_sub_;

  // Function to compute M(q)
  Eigen::Matrix2d compute_M(double theta);

  //function to compute c( q, q_dot)
  Eigen::Matrix2d compute_C(double theta, double theta_dot);

  // Function to compute G(q)
  Eigen::Vector2d compute_G(double theta);

  // Function to compute accelerations (q_ddot)
  Eigen::Vector2d compute_acceleration(const Eigen::Vector2d &q, const Eigen::Vector2d &q_dot, double force);

  //void publish to joint states
  void publish(Eigen::Vector2d &q_ddot);

  //void simulate
  void simulate(void);




};

}

#endif