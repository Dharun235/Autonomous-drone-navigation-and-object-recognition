#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "darknet.h"

class YOLONode : public rclcpp::Node
{
public:
    YOLONode() : Node("yolo_node")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&YOLONode::image_callback, this, std::placeholders::_1));

        // Initialize YOLO
        net_ = load_network("path_to_cfg_file", "path_to_weights_file", 0);
        meta_ = get_metadata("path_to_data_file");
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
        free_detections(dets, meta_.classes);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    network *net_;
    metadata meta_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YOLONode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
