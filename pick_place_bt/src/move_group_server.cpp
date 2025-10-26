#include "pick_place_bt/move_group_server.hpp"

namespace move_group_server
{
    MoveGroupService::MoveGroupService(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        auto goal_position_tolerance = move_group_->getGoalPositionTolerance();
        auto goal_orientation_tolerance = move_group_->getGoalOrientationTolerance();
        auto end_effector = move_group_->getEndEffector();
        auto planning_frame = move_group_->getPlanningFrame();
        RCLCPP_INFO(node->get_logger(), "Goal position tolerance is %f", goal_position_tolerance);
        RCLCPP_INFO(node->get_logger(), "Goal orientation tolerance is %f", goal_orientation_tolerance);
        RCLCPP_INFO(node_->get_logger(), "End effector is %s", end_effector.c_str());
        RCLCPP_INFO(node_->get_logger(), "Planning frame is %s", planning_frame.c_str());
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

        // geometry_msgs::msg::PoseStamped target_pose;
        // target_pose.header.frame_id = move_group_->getPlanningFrame();
        // target_pose.header.frame_id = "J6";
        // target_pose.header.stamp = node_->now();
        // target_pose.pose = request->target_pose;
        RCLCPP_INFO(node_->get_logger(), "Received move_to request to position: x=%f, y=%f, z=%f, wx=%f, wy=%f, wz=%f, w=%f",
                    request->target_pose.position.x,
                    request->target_pose.position.y,
                    request->target_pose.position.z,
                    request->target_pose.orientation.x,
                    request->target_pose.orientation.y,
                    request->target_pose.orientation.z,
                    request->target_pose.orientation.w);
        move_group_->clearPoseTarget();
        move_group_->setPoseTarget(request->target_pose);
        // move_group_->setPositionTarget(request->target_pose.position.x,
        //                               request->target_pose.position.y,
        //                               request->target_pose.position.z);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(node_->get_logger(), "Plan created successfully");
            move_group_->execute(my_plan);
            response->success = true;
            response->message = "Robot moved to target pose successfully.";
        }
        else 
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan trajectory");
            response->success = false;
            response->message = "Failed to move robot to target pose.";
        }

        // if (success)
        // {
        //     RCLCPP_INFO(node_->get_logger(), "Robot moved to the target pose successfully.");
        // }
        // else
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "Failed to move the robot to the target pose.");
        // }
        // RCLCPP_INFO(node_->get_logger(), "Received move_to request to position: x=%f, y=%f, z=%f",
        // request->position.x, request->position.y, request->position.z);

        // Here you would add the logic to move the robot using MoveIt!
        // For now, we just simulate a successful move.
        response->success = true;
        RCLCPP_INFO(node_->get_logger(), "MoveTo request handled successfully.");
    }

    geometry_msgs::msg::PoseStamped MoveGroupService::get_current_pose()
    {
        return move_group_->getCurrentPose("J6");
    }

    std::vector<double> MoveGroupService::get_current_joint_values() {
        return move_group_->getCurrentJointValues();
    }

} // namespace move_group_server

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_group_server_node",
                                               rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
                                               rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    // std::thread([&executor]() {executor.spin();}).detach();

    // executor.spin_once();

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("move_group_server_node");

    auto move_group_service = std::make_shared<move_group_server::MoveGroupService>(node);

    auto pose = move_group_service->get_current_pose();
    RCLCPP_INFO(logger, "Pose value is x: %f", pose.pose.position.x);
    RCLCPP_INFO(logger, "Pose value is y: %f", pose.pose.position.y);
    RCLCPP_INFO(logger, "Pose value is z: %f", pose.pose.position.z);
    RCLCPP_INFO(logger, "Pose value is wx: %f", pose.pose.orientation.x);
    RCLCPP_INFO(logger, "Pose value is wy: %f", pose.pose.orientation.y);
    RCLCPP_INFO(logger, "Pose value is wz: %f", pose.pose.orientation.z);
    RCLCPP_INFO(logger, "Pose value is w: %f", pose.pose.orientation.w);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}