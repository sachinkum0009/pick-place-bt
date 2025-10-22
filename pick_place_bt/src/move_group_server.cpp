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
        RCLCPP_INFO(node_->get_logger(), "Received move_to request to position: x=%f, y=%f, z=%f",
                    request->position.x, request->position.y, request->position.z);

        // Here you would add the logic to move the robot using MoveIt!
        // For now, we just simulate a successful move.
        response->success = true;
        RCLCPP_INFO(node_->get_logger(), "MoveTo request handled successfully.");
    }

    void MoveGroupService::execute()
    {
        RCLCPP_INFO(node_->get_logger(), "Executing MoveGroupService...");

        // Example: Move to a predefined pose
        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // if (success)
        // {
        //     RCLCPP_INFO(node_->get_logger(), "Planning successful, executing...");
        //     move_group_.execute(my_plan);
        //     RCLCPP_INFO(node_->get_logger(), "Movement executed.");
        // }
        // else
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "Planning failed.");
        // }
    }
} // namespace move_group_server


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_group_server_node",
                                               rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    // Create a ROS logger
    auto const logger = rclcpp::get_logger("move_group_server_node");


    // auto move_group_service = std::make_shared<move_group_server::MoveGroupService>();
    // move_group_service->execute();
    rclcpp::shutdown();
    return 0;
}