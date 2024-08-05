#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "ORB_SLAM2/System.h"

class ORBSLAM2Node : public rclcpp::Node
{
public:
    ORBSLAM2Node() : Node("orb_slam2_node")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ORBSLAM2Node::image_callback, this, std::placeholders::_1));

        // Initialize ORB SLAM2 system
        orb_slam_ = std::make_shared<ORB_SLAM2::System>(
            "path_to_vocabulary_file", "path_to_settings_file", ORB_SLAM2::System::MONOCULAR, true);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Pass frame to ORB SLAM2
        orb_slam_->TrackMonocular(cv_ptr->image, msg->header.stamp.sec);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::shared_ptr<ORB_SLAM2::System> orb_slam_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ORBSLAM2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
