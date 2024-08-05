#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "darknet.h"
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>

class YOLONode : public rclcpp::Node
{
public:
    YOLONode() : Node("yolo_node"), file_("detected_fruits.txt")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&YOLONode::image_callback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/slam/pose", 10,
            std::bind(&YOLONode::pose_callback, this, std::placeholders::_1));

        // Initialize YOLO
        net_ = load_network("/path/to/yolov3.cfg", "/path/to/yolov3.weights", 0);
        meta_ = get_metadata("/path/to/coco.data");
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

        // Pass frame to YOLO
        image im = mat_to_image(cv_ptr->image);
        network_predict_image(net_, im);
        detection *dets = get_network_boxes(net_, im.w, im.h, 0.5, 0.5, 0, 1, NULL, 0, NULL);
        draw_detections(cv_ptr->image, dets, meta_.classes, 0.5);
        
        for (int i = 0; i < dets->total; ++i)
        {
            if (dets[i].prob[0] > 0.5)
            {
                std::string label = meta_.names[dets[i].prob[0]];
                if (label == "fruit") // Adjust label as per your data file
                {
                    file_ << "Fruit detected at: "
                          << "x: " << current_pose_.pose.position.x
                          << " y: " << current_pose_.pose.position.y
                          << " z: " << current_pose_.pose.position.z
                          << std::endl;
                }
            }
        }

        free_detections(dets, meta_.classes);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    network *net_;
    metadata meta_;
    geometry_msgs::msg::PoseStamped current_pose_;
    std::ofstream file_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YOLONode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
