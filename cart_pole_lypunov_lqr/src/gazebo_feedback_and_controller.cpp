#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <stdio.h>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#define PI 3.14159

// ANSI escape codes for colors
#define GREEN_TEXT "\033[0;32m"
#define BLUE_TEXT "\033[0;34m"
#define RESET_COLOR "\033[0m"

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

            this->declare_parameter("K", rclcpp::PARAMETER_DOUBLE_ARRAY);
            this->declare_parameter("ke", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("kv", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("kx", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("kdelta", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("lqr_transition_angle", rclcpp::PARAMETER_DOUBLE);

                        
            try {
                K_ = Eigen::Vector4d(this->get_parameter("K").as_double_array().data());
                ke_ = this->get_parameter("ke").as_double();
                kv_ = this->get_parameter("kv").as_double();
                kx_ = this->get_parameter("kx").as_double();
                kdelta_ = this->get_parameter("kdelta").as_double();
                lqr_transition_angle_ = this->get_parameter("lqr_transition_angle").as_double();

                
            RCLCPP_WARN(this->get_logger(), "K: %.4f, %.4f, %.4f, %.4f", K_(0), K_(1), K_(2), K_(3));
            RCLCPP_WARN(this->get_logger(), "ke: %.4f", ke_);
            RCLCPP_WARN(this->get_logger(), "kv: %.4f", kv_);
            RCLCPP_WARN(this->get_logger(), "kx: %.4f", kx_);
            RCLCPP_WARN(this->get_logger(), "kdelta: %.4f", kdelta_);
            RCLCPP_WARN(this->get_logger(), "lqr_transition_angle_: %.4f", lqr_transition_angle_);

            } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
                throw e;
            }

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

            double switching_range = lqr_transition_angle_;


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

        double f;

        if((slider_it != msg.name.end()) || (swinger_it != msg.name.end()))
        {
              // Define the range boundaries
              double lower_limit = PI - switching_range;
              double upper_limit = PI + switching_range;

                  // Normalize swinger_position to [0, 2*PI]
              double normalized_position = fmod(swinger_position, 2 * PI);

                  // If the normalized position is negative, adjust it by adding 2*PI
              if (normalized_position < 0) {
                  normalized_position += 2 * PI;
              }
                  
         //RCLCPP_INFO(this->get_logger(), "normalised_position: %.4f", normalized_position);

         //Before the pendulum is in the homoclinic orbit
        if( normalized_position < lower_limit || normalized_position > upper_limit)
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
    /*--------------------------------------------------------------------------------*/
    //THIS SHOULD BE ADDED TO A YAML FILE
    double ke = ke_;  // reduce this since it is on the denominator
    double kv = kv_; //leave this constant since it is both in numerator and denominator
    double kx = kx_; //increase to make more aggressive since it is on the numerator
    double kdelta = kdelta_; //increase since it is at the numerator
     /*-------------------------------------------------------------------------------*/

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
    f = numerator / denominator;
    RCLCPP_INFO(this->get_logger(), BLUE_TEXT"F: %.4f", f);
    //RCLCPP_INFO(this->get_logger(), "Force: %.4f Numerator: %.4f Denominator: %.4f Energy: %.4f", f, numerator, denominator, E);
    }

    else 
    {
       double gains_[4] = { K_(0), K_(1), K_(2), K_(3)};

       double from_output = (gains_[0]*swinger_position);
       double first = (gains_[1]*swinger_velocity) + (gains_[2]*slider_position) +(gains_[3]*slider_velocity);

       double final_gain = (from_output - first);
       f = final_gain;
       RCLCPP_INFO(this->get_logger(), GREEN_TEXT"F: %.4f", f);

    }

    auto message = std_msgs::msg::Float64MultiArray();
    message.data.push_back(f);
    publisher_->publish(message);
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_; // effort controller publisher
    Eigen::Vector4d K_;
    double ke_;
    double kv_;
    double kx_;
    double kdelta_;
    double lqr_transition_angle_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateSubscriber>());
  rclcpp::shutdown();
  return 0;
}