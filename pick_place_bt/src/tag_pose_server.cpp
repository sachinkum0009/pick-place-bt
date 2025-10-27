#include "pick_place_bt/tag_pose_server.hpp"

namespace tag_pose_server
{
    TagPoseServer::TagPoseServer(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        service_ = node_->create_service<TagPose>("tag_pose", std::bind(&TagPoseServer::handle_tag_pose_request, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(node_->get_logger(), "TagPoseServer initialized.");
    }

    TagPoseServer::~TagPoseServer()
    {
        RCLCPP_INFO(node_->get_logger(), "TagPoseServer shutting down.");
    }

    void TagPoseServer::handle_tag_pose_request(const std::shared_ptr<TagPose::Request> request,
                                                std::shared_ptr<TagPose::Response> response)
    {
        RCLCPP_INFO(node_->get_logger(), "Received tag_pose request for ID: %d", request->id);
        try
        {
            auto t = tf_buffer_->lookupTransform("tag"+std::to_string(request->id), "J6", tf2::TimePointZero, tf2::durationFromSec(1.0));
            response->success = true;
            response->pose.position.x = t.transform.translation.x;
            response->pose.position.y = t.transform.translation.y;
            response->pose.position.z = t.transform.translation.z;
            response->pose.orientation.x = t.transform.rotation.x;
            response->pose.orientation.y = t.transform.rotation.y;
            response->pose.orientation.z = t.transform.rotation.z;
            response->pose.orientation.w = t.transform.rotation.w;
            RCLCPP_INFO(node_->get_logger(), "Tag pose response sent.");
        }
        catch(const tf2::TransformException & ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not get transform: %s", ex.what());
            response->success = false;
            return;
        }

    }
} // namespace tag_pose_server

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tag_pose_server_node");
    auto tag_pose_server = std::make_shared<tag_pose_server::TagPoseServer>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
