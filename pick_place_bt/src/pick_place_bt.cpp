#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_ros2/bt_action_node.hpp"

#include "pick_place_bt/set_bool_node.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("pick_place_bt_node");
    RCLCPP_INFO(nh->get_logger(), "Pick and Place Behavior Tree Node Started.");
    BT::BehaviorTreeFactory factory;

    // version with default port
    factory.registerNodeType<SetBoolService>("SetBoolA", BT::RosNodeParams(nh, "robotA/set_bool"));

    // version without default port
    factory.registerNodeType<SetBoolService>("SetBool", BT::RosNodeParams(nh));

    // namespace version
    factory.registerNodeType<SetRobotBoolService>("SetRobotBool", nh, "set_bool");

    // Load XML from package share and create tree from file
    std::string pkg_share;
    try {
        pkg_share = ament_index_cpp::get_package_share_directory("pick_place_bt");
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(nh->get_logger(), "Could not find package share: %s", ex.what());
        return 1;
    }

    std::string xml_path = pkg_share + "/behavior_trees/service_bool.xml";

    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(xml_path);
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(nh->get_logger(), "Failed to create tree from file '%s': %s", xml_path.c_str(), ex.what());
        return 1;
    }

    // auto tree = factory.createTreeFromText(xml_text);

    tree.tickWhileRunning();

    rclcpp::shutdown();
    return 0;
}
