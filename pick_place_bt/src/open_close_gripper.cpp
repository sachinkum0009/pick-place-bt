#include <behaviortree_ros2/bt_service_node.hpp>
#include <bcap_service_interfaces/srv/gripper.hpp>

using BcapGripper = bcap_service_interfaces::srv::Gripper;
using namespace BT;

class MoveArmService : public RosServiceNode<BcapGripper>
{
public:
    MoveArmService(const std::string &name, const NodeConfig &conf, const RosNodeParams &params)
        : RosServiceNode<BcapGripper>(name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("command", "open", "Command to open or close the gripper"),
            InputPort<int>("speed", 10, "Speed to open or close the gripper")
            // InputPort<unsigned>("timeout", 5000, "Timeout to wait for service server (ms)")
        });
    }

    bool setRequest(typename Request::SharedPtr &request) override
    {
        std::string command;
        if (!getInput<std::string>("command", command))
        {
            RCLCPP_ERROR(logger(), "Failed to get input port 'command'");
            return false;
        }

        int speed;

        if (!getInput<int>("speed", speed))
        {
            RCLCPP_ERROR(logger(), "Failed to get input port 'speed'");
            return false;
        }

        if (command == "open")
        {
            request->value = 30;
            request->speed = speed;
        }
        else if (command == "close")
        {
            request->value = 19;
            request->speed = speed;
        }
        else
        {
            RCLCPP_ERROR(logger(), "Invalid command: %s", command.c_str());
            return false;
        }
        return true;
    }

    NodeStatus onResponseReceived(const typename Response::SharedPtr &response) override
    {
        RCLCPP_INFO(logger(), "Gripper command executed with result: %s", response->message.c_str());
        return NodeStatus::SUCCESS;
    }

    virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "Service call failed with error code: %d", error);
        return NodeStatus::FAILURE;
    }
};