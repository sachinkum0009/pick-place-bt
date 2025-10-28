#ifndef PICK_PLACE_BT_MOVE_GROUP_SERVER_HPP
#define PICK_PLACE_BT_MOVE_GROUP_SERVER_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <pick_place_interface/srv/move_to.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace move_group_server
{
    class MoveGroupService
    {
    public:
        MoveGroupService(std::shared_ptr<rclcpp::Node> node);
        ~MoveGroupService();
        void handle_move_to(const std::shared_ptr<pick_place_interface::srv::MoveTo::Request> request,
                            std::shared_ptr<pick_place_interface::srv::MoveTo::Response> response);
        geometry_msgs::msg::PoseStamped get_current_pose();
        std::vector<double> get_current_joint_values();
        bool wait_for_current_state(double timeout_seconds);
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    private:
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Service<pick_place_interface::srv::MoveTo>::SharedPtr service_;
    };
} // namespace move_group_server


#endif  // PICK_PLACE_BT_MOVE_GROUP_SERVER_HPP
