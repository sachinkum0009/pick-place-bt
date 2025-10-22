#ifndef PICK_PLACE_BT_MOVE_GROUP_SERVER_HPP
#define PICK_PLACE_BT_MOVE_GROUP_SERVER_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <pick_place_interface/srv/move_to.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace move_group_server
{
    class MoveGroupService
    {
    public:
        MoveGroupService(std::shared_ptr<rclcpp::Node> node);
        ~MoveGroupService();
        void handle_move_to(const std::shared_ptr<pick_place_interface::srv::MoveTo::Request> request,
                            std::shared_ptr<pick_place_interface::srv::MoveTo::Response> response);
    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        rclcpp::Service<pick_place_interface::srv::MoveTo>::SharedPtr service_;
    };
} // namespace move_group_server


#endif  // PICK_PLACE_BT_MOVE_GROUP_SERVER_HPP
