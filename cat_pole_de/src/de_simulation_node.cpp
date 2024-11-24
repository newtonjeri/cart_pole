#include <cat_pole_de/de_simulation_node.hpp>

#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace cart_pole_de
{

DeSimulationNode::DeSimulationNode(const rclcpp::NodeOptions & options)
: Node("cart_pole_de_simulation", options)
{
    // Create the simulation node
    this->declare_parameter("initial_conditions.position", 0.0);
    this->declare_parameter("initial_conditions.theta", 0.0);

    this->declare_parameter("initial_conditions.dposition", 0.0);
    this->declare_parameter("initial_conditions.dtheta", 0.0);

    this->declare_parameter("initial_conditions.ddposition", 0.0);
    this->declare_parameter("initial_conditions.ddtheta", 0.0);

    this->declare_parameter("initial_conditions.tau1", 0.0);
    this->declare_parameter("initial_conditions.tau2", 0.0);

    this->declare_parameter("initial_conditions.simulation_dt", 0.0);

    position_ = this->get_parameter("initial_conditions.position").as_double();
    theta_ = this->get_parameter("initial_conditions.theta").as_double();

    dposition_ = this->get_parameter("initial_conditions.dposition").as_double();
    dtheta_ = this->get_parameter("initial_conditions.dtheta").as_double();

    ddposition_ = this->get_parameter("initial_conditions.ddposition").as_double();
    ddtheta_ = this->get_parameter("initial_conditions.ddtheta").as_double();

    tau1_ = this->get_parameter("initial_conditions.tau1").as_double();
    tau2_ = this->get_parameter("initial_conditions.tau2").as_double();

    this->declare_parameter("simulation_dt", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_torque", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_velocity_slider_1", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_velocity_continuous_revolute_1", rclcpp::PARAMETER_DOUBLE);
 
    m1_ = 2.7582989593438584919; m2_ = 0.96730709424135585817;  l1_ = 500/2;


    try {
        dt_ = this->get_parameter("simulation_dt").as_double();
        max_torque_ = this->get_parameter("max_torque").as_double();
        max_velocity_slider_1_ = this->get_parameter("max_velocity_slider_1").as_double();
        max_velocity_continuous_revolute_1_ = this->get_parameter("max_velocity_continuous_revolute_1").as_double();

    } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
        throw e;
    }



  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  simulation_timer_ = this->create_wall_timer( std::chrono::duration<double>(dt_), std::bind(&DeSimulationNode::simulate, this));

}


  // Function to compute M(q)
  Eigen::Matrix2d DeSimulationNode::compute_M(double theta)
  {
    Eigen::Matrix2d M_matrix;
    M_matrix(0, 0) = m1_ + m2_;
    M_matrix(0, 1) = m2_ * l1_ * cos(theta);
    M_matrix(1, 0) = m2_ * l1_ * cos(theta);
    M_matrix(1, 1) = m2_ * l1_ * l1_;
    return M_matrix;
  }

    // Function to compute C(q, q_dot)
    Eigen::Matrix2d DeSimulationNode::compute_C(double theta, double theta_dot) {
        Eigen::Matrix2d C_matrix = Eigen::Matrix2d::Zero();
        C_matrix(0, 1) = -m2_ * l1_ * sin(theta) * theta_dot;
        return C_matrix;
    }

    // Function to compute G(q)
    Eigen::Vector2d DeSimulationNode::compute_G(double theta) {
        Eigen::Vector2d G_vector;
        G_vector(0) = 0;
        G_vector(1) = -m2_ * g_ * l1_ * sin(theta);
        return G_vector;
    }

    // Function to compute accelerations (q_ddot)
    Eigen::Vector2d DeSimulationNode::compute_acceleration(const Eigen::Vector2d &q, const Eigen::Vector2d &q_dot, double force) {
        //double x = q(0);
        double theta = q(1);
        //double x_dot = q_dot(0);
        double theta_dot = q_dot(1);

        // Compute matrices
        Eigen::Matrix2d M_matrix = compute_M(theta);
        Eigen::Matrix2d C_matrix = compute_C(theta, theta_dot);
        Eigen::Vector2d G_vector = compute_G(theta);
        Eigen::Vector2d tau(force, 0);

        // Compute acceleration
        Eigen::Vector2d q_ddot = M_matrix.inverse() * (tau - C_matrix * q_dot - G_vector);
        return q_ddot;
    }

    void DeSimulationNode::publish(Eigen::Vector2d &q_ddot)
    {
        // Integrate using Euler's method
        q_dot_ += q_ddot * dt_;  // Update velocity
        q_ += q_dot_ * dt_;       // Update position

        sensor_msgs::msg::JointState joint_state_msg;

        joint_state_msg.header.stamp.sec = current_time_;
        joint_state_msg.header.stamp.nanosec = (current_time_ - (int)current_time_) * 1000000000.0;

        
        joint_state_msg.name.push_back("slider 1");
        joint_state_msg.position.push_back(q_(0));
        joint_state_msg.velocity.push_back(q_dot_(0));

        joint_state_msg.name.push_back("continuous_revolute 1");
        joint_state_msg.position.push_back(q_(1));
        joint_state_msg.velocity.push_back(q_dot_(1));

        //print theta
         //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "theta1: %f  : theta2: %f", theta1_, theta2_);
         joint_state_pub_->publish(joint_state_msg);

    }

    void DeSimulationNode::simulate(void)
    { 
        double force = 0.0;
        Eigen::Vector2d q_ddot = compute_acceleration( q_, q_dot_, force);
        publish(q_ddot);

        // Increment time
        current_time_ += dt_;

    }
    
} // namespace furuta_pendulum_de

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cart_pole_de::DeSimulationNode)