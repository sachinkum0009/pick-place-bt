#ifndef FOUNDATION_POSE_SERVER_HPP
#define FOUNDATION_POSE_SERVER_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace foundation_pose_server
{
    class FoundationPoseServer
    {
    public:
        FoundationPoseServer(std::shared_ptr<rclcpp::Node> node);
        ~FoundationPoseServer();

    private:
        std::shared_ptr<rclcpp::Node> node_;
    };
} // namespace foundation_pose_server

#endif // FOUNDATION_POSE_SERVER_HPP