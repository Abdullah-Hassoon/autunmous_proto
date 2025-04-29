#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

const float threshold = 0.5;
const float angle_increment = 0.009824 ;

class AutomaticBrake : public rclcpp::Node
{
    public:
        AutomaticBrake(): Node("automatic_brake"), count_(0)
        {
            vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist> (
                                "/diff_drive_contoller/cmd_vel_unstamped", 10);
            
            vel_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                                "/diff_drive_controller/odom", 100, std::bind(
                                    &AutomaticBrake::velocity_callback, this, std::placeholders::_1));

            scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                                "/scan", 100, std::bind(
                                    &AutomaticBrake::scan_callback, this, std::placeholders::_1));

    }              

    private:

        void velocity_callback(const nav_msgs::msg::Odometry & msg){
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = 1.0;
            vel_msg.angular.z = 1.0;
            
            auto velocity = msg.twist.twist.linear.x;
            std::vector<float> iTTC = calculate_iTTC(velocity, scan_ranges_);
            brake = is_brake(iTTC);
            if(brake)
            {
                std::cout << "Braking!" << std::endl;
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
                vel_publisher_->publish(vel_msg);
            }
            brake = false;
        }

        void scan_callback(const sensor_msgs::msg::LaserScan & msg){
            // process the scan data
            for (size_t i = 0; i < msg.ranges.size(); ++i) {
                scan_ranges_[i] = msg.ranges[i];
            }
        }

        bool is_brake(std::vector<float> iTTC){
            // check if any iTTC is less than 0.5
            for (size_t i = 0; i < iTTC.size(); ++i) {
                if (iTTC[i] < threshold) {
                    return true;
                }
            }
            return false;
        }

        std::vector<float> calculate_iTTC(float velocity, const std::vector<float> &scan_ranges){
            // map the velocity to iTTC for each angle
            std::vector<float> iTTC(scan_ranges.size());
            float theta = -3.14;
            for (size_t i = 0; i < scan_ranges.size(); ++i) {
                float cos_theta = std::cos(theta);
                if (scan_ranges[i] > 0.0 && scan_ranges[i] < 10.0) {
                    if(cos_theta > 0.01)
                        iTTC[i] = scan_ranges[i] / (velocity * cos_theta);
                    else
                        iTTC[i] = std::numeric_limits<float>::infinity(); 
                } 
                else {
                    iTTC[i] = std::numeric_limits<float>::infinity(); // or some large value
                }
                theta += angle_increment;
            }

            for(float & i : iTTC){
                std::cout << i << " ";
            }
            return iTTC;
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vel_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
        size_t count_;
        bool brake = false;
        std::vector<float> scan_ranges_{360, 0.0}; // Initialize with 360 elements

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutomaticBrake>());
    rclcpp::shutdown();
    return 0;
}