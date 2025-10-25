#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include "open_close_gripper.cpp"
#include "delay_node.cpp"
#include "move_to.cpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("gripper_bt_node");
    RCLCPP_INFO(nh->get_logger(), "Gripper Behavior Tree Node Started.");
    
    BT::BehaviorTreeFactory factory;
    
    // Configure ROS node parameters with longer timeouts
    BT::RosNodeParams params;
    params.nh = nh;
    params.default_port_value = "/gripper_service";
    params.wait_for_server_timeout = std::chrono::milliseconds(5000);  // 5 seconds to wait for service
    params.server_timeout = std::chrono::milliseconds(10000);  // 10 seconds for service call
    
    // Register the gripper node with the service name and custom params
    factory.registerNodeType<MoveArmService>("OpenCloseGripper", params);
    
    // Register our custom wait/delay node
    factory.registerNodeType<BT::WaitNode>("Wait");

    BT::RosNodeParams move_to_params;
    move_to_params.nh = nh;
    move_to_params.default_port_value = "/move_to";
    move_to_params.wait_for_server_timeout = std::chrono::milliseconds(5000);  // 5 seconds to wait for service as the robot is moving very slowly
    move_to_params.server_timeout = std::chrono::milliseconds(50000);  // 50 seconds for service call

    // Register our custom move_to node
    factory.registerNodeType<MoveToService>("MoveTo", move_to_params);
    
    // Load XML from package share and create tree from file
    std::string pkg_share;
    try {
        pkg_share = ament_index_cpp::get_package_share_directory("pick_place_bt");
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(nh->get_logger(), "Could not find package share: %s", ex.what());
        return 1;
    }
    
    std::string xml_path = pkg_share + "/behavior_trees/gripper_control.xml";
    
    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(xml_path);
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(nh->get_logger(), "Failed to create tree from file '%s': %s", 
                     xml_path.c_str(), ex.what());
        return 1;
    }
    
    // Give some time for service connections to establish
    rclcpp::sleep_for(std::chrono::seconds(1));

    
    // Execute the behavior tree
    tree.tickWhileRunning();
    
    RCLCPP_INFO(nh->get_logger(), "Gripper Behavior Tree execution completed.");
    
    rclcpp::shutdown();
    return 0;
}
