#ifndef PICK_PLACE_BT_TAG_POSE_SERVER_HPP
#define PICK_PLACE_BT_TAG_POSE_SERVER_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include "pick_place_interface/srv/tag_pose.hpp"

using TagPose = pick_place_interface::srv::TagPose;

namespace tag_pose_server
{
    class TagPoseServer
    {
    public:
        TagPoseServer(std::shared_ptr<rclcpp::Node> node);
        ~TagPoseServer();

    private:
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Service<TagPose>::SharedPtr service_;
        void handle_tag_pose_request(const std::shared_ptr<TagPose::Request> request,
                                     std::shared_ptr<TagPose::Response> response);
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    };
} // namespace tag_pose_server

#endif // PICK_PLACE_BT_TAG_POSE_SERVER_HPP