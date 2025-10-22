#include "pick_place_bt/move_group_server.hpp"

namespace move_group_server
{
    MoveGroupService::MoveGroupService(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "manipulator");
        service_ = node_->create_service<pick_place_interface::srv::MoveTo>("move_to", std::bind(&MoveGroupService::handle_move_to, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(node_->get_logger(), "MoveGroupService initialized.");
    }

    MoveGroupService::~MoveGroupService()
    {
        RCLCPP_INFO(node_->get_logger(), "MoveGroupService shutting down.");
    }

    void MoveGroupService::handle_move_to(const std::shared_ptr<pick_place_interface::srv::MoveTo::Request> request,
                                          std::shared_ptr<pick_place_interface::srv::MoveTo::Response> response)
    {

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.pose = request->target_pose;
        move_group_->setPoseTarget(target_pose);

        bool success = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            response->success = true;
            response->message = "Robot moved to target pose successfully.";
            RCLCPP_INFO(node_->get_logger(), "Robot moved to the target pose successfully.");
        }
        else
        {
            response->success = false;
            response->message = "Failed to move robot to target pose.";
            RCLCPP_ERROR(node_->get_logger(), "Failed to move the robot to the target pose.");
        }
        // RCLCPP_INFO(node_->get_logger(), "Received move_to request to position: x=%f, y=%f, z=%f",
        // request->position.x, request->position.y, request->position.z);

        // Here you would add the logic to move the robot using MoveIt!
        // For now, we just simulate a successful move.
        response->success = true;
        RCLCPP_INFO(node_->get_logger(), "MoveTo request handled successfully.");
    }

} // namespace move_group_server

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_group_server_node",
                                               rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    // Create a ROS logger
    auto const logger = rclcpp::get_logger("move_group_server_node");

    auto move_group_service = std::make_shared<move_group_server::MoveGroupService>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}