#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PIDControllerNode : public rclcpp::Node
{
public:
    PIDControllerNode() : Node("pid_controller_node")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/slam/pose", 10,
            std::bind(&PIDControllerNode::pose_callback, this, std::placeholders::_1));

        // Define start and goal positions (example coordinates)
        start_x_ = 0.0; start_y_ = 0.0; start_z_ = 1.0;
        goal_x_ = 10.0; goal_y_ = 10.0; goal_z_ = 1.0;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PIDControllerNode::control_loop, this));
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
    }

    void control_loop()
    {
        auto cmd_msg = geometry_msgs::msg::Twist();
        
        // PID control logic here to move from start to goal
        float error_x = goal_x_ - current_pose_.pose.position.x;
        float error_y = goal_y_ - current_pose_.pose.position.y;
        float error_z = goal_z_ - current_pose_.pose.position.z;

        // Example simple proportional controller (you can implement a full PID here)
        cmd_msg.linear.x = 0.1 * error_x;
        cmd_msg.linear.y = 0.1 * error_y;
        cmd_msg.linear.z = 0.1 * error_z;

        // Publish the control command
        cmd_vel_pub_->publish(cmd_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped current_pose_;
    float start_x_, start_y_, start_z_;
    float goal_x_, goal_y_, goal_z_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
