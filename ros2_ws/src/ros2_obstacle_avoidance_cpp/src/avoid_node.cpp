#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

using std::placeholders::_1;

class AvoidNode : public rclcpp::Node {
public:
    AvoidNode() : Node("obstacle_avoidance") {
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&AvoidNode::pose_callback, this, _1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AvoidNode::move, this));
    }

private:
    turtlesim::msg::Pose::SharedPtr last_pose_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        last_pose_ = msg;
    }

    void move() {
        auto cmd = geometry_msgs::msg::Twist();
        if (!last_pose_) return;

        const double border_buffer = 1.0;
        const double max_x = 11.0 - border_buffer;
        const double min_x = 0.0 + border_buffer;
        const double max_y = 11.0 - border_buffer;
        const double min_y = 0.0 + border_buffer;
        
        bool near_border = false;
        double turn_direction = 0.0;
        
        if (last_pose_->x < min_x) {
            near_border = true;
            turn_direction = -1.0;
        } else if (last_pose_->x > max_x) {
            near_border = true;
            turn_direction = 1.0;
        }
        
        if (last_pose_->y < min_y) {
            near_border = true;
            if (std::sin(last_pose_->theta) < 0) {
                turn_direction = 1.0;
            }
        } else if (last_pose_->y > max_y) {
            near_border = true;
            if (std::sin(last_pose_->theta) > 0) {
                turn_direction = 1.0;
            }
        }
        
        if (near_border) {
            cmd.angular.z = 1.5 * turn_direction;
            cmd.linear.x = 0.5;
        } else {
            cmd.linear.x = 2.0;
            cmd.angular.z = 0.0;
        }

        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AvoidNode>());
    rclcpp::shutdown();
    return 0;
}
