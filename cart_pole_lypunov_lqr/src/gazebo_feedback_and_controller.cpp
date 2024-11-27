#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

class JointStateSubscriber : public rclcpp::Node
{
  public:
    JointStateSubscriber()
    : Node("state_space_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&JointStateSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("effort_controller/commands", 10);

    }

  private:
    void topic_callback(const sensor_msgs::msg::JointState & msg)
    {
        // Find the indices of the "slider" and "swinger" joints
        auto slider_it = std::find(msg.name.begin(), msg.name.end(), "slider_1");
        auto swinger_it = std::find(msg.name.begin(), msg.name.end(), "continuous_revolute_1");

        
            double slider_position;
            double slider_velocity;

            
            double swinger_position;
            double swinger_velocity;


        if (slider_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), slider_it);
            slider_position = msg.position[index];
            slider_velocity = msg.velocity[index];

            //RCLCPP_INFO(this->get_logger(), "Slider  -> Position: %.2f, Velocity: %.2f", slider_position, slider_velocity);
        }

        else
        {
            RCLCPP_WARN(this->get_logger(), "Slider joint not found in the message.");
        }

        if (swinger_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger_it);
            swinger_position = msg.position[index];
            swinger_velocity = msg.velocity[index];

            //RCLCPP_INFO(this->get_logger(), "Swinger -> Position: %.2f, Velocity: %.2f", swinger_position, swinger_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger joint not found in the message.");
        }

        if((slider_it != msg.name.end()) || (swinger_it != msg.name.end()))
        {

                  // Define system parameters
            double M = 2.7582989593438584919;  // Mass of cart (kg)
            double m = 0.96730709424135585817;  // Mass of pendulum (kg)
            double l = 0.5/2;  // Length of pendulum (m)
            double g = 9.81; // Gravitational acceleration (m/s^2)

            // State variables
            double x = slider_position;        // Cart position (m)
            ///subracting pi from the input to invert it
            double theta = 3.14159265358979323846 - swinger_position;    // Pendulum angle (rad)
            double dx = slider_velocity;       // Cart velocity (m/s)
            double dtheta = swinger_velocity;   // Pendulum angular velocity (rad/s)


    // Mass matrix M(q)
    Eigen::Matrix2d Mq;
    Mq << M + m, m * l * cos(theta), 
          m * l * cos(theta), m * l * l;

    // Coriolis/Centrifugal forces
    Eigen::Matrix2d Cq;

    Cq << 0, -m * l * sin(theta) * dtheta,
          0, 0;

    // Gravity vector G(q)
    Eigen::Vector2d Gq;
    Gq << 0,
          -m * g * l * sin(theta);

    

    // Control parameters
    
    double ke = 10.0;
    double kv = 100.0;
    double kx = 100.0;
    double kdelta = 0.01;

    // Forming z
    double z3 = cos(theta);

    //forming zeds
    Eigen::Vector2d zeds;
    zeds << dx, dtheta;

    // Energy (E)
    double E = 0.5 * zeds.transpose() * Mq * zeds + m * g * l * (z3 - 1);

    // Numerator
    double numerator = kv * m * sin(theta) * (g * cos(theta) - l * dtheta * dtheta) - (M + m * sin(theta) * sin(theta)) * (kx * x + kdelta * dx);
    
    
    //RCLCPP_INFO(this->get_logger(), "Part 1: %.8f Part2: %.8f, angle: %f, positin: %f, position_d: %f", kv * m * sin(theta) * (g * cos(theta) - l * dtheta * dtheta), -(M + m * sin(theta) * sin(theta)) * (kx * x + kdelta * dx), theta, x, slider_position);                                //0                             //0                      //M
    // Denominator
    //RCLCPP_INFO(this->get_logger(), "Position: %.4f ", slider_position);

    double denominator = kv + (M + m * sin(theta) * sin(theta)) * ke * E;

    // Control input
    double f = numerator / denominator;
    RCLCPP_INFO(this->get_logger(), "Force: %.4f Numerator: %.4f Denominator: %.4f Energy: %.4f", f, numerator, denominator, E);
    //RCLCPP_INFO(this->get_logger(), "Force: %.4f Numerator: %.4f Denominator: %.4f Energy: %.4f", f, numerator, denominator, E);

    auto message = std_msgs::msg::Float64MultiArray();
    message.data.push_back(f*10000);
    publisher_->publish(message);
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_; // effort controller publisher
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateSubscriber>());
  rclcpp::shutdown();
  return 0;
}