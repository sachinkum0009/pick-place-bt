#include <behaviortree_ros2/bt_service_node.hpp>
#include <pick_place_interface/srv/move_to.hpp>

using MoveTo = pick_place_interface::srv::MoveTo;
using namespace BT;

class MoveToService : public RosServiceNode<MoveTo>
{
public:
    MoveToService(const std::string &name, const NodeConfig &conf, const RosNodeParams &params)
        : RosServiceNode<MoveTo>(name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<double>("x", 0.0, "X coordinate"),
            InputPort<double>("y", 0.0, "Y coordinate"),
            InputPort<double>("z", 0.0, "Z coordinate"),
            InputPort<double>("wx", 0.0, "quater X coordinate"),
            InputPort<double>("wy", 0.0, "quater Y coordinate"),
            InputPort<double>("wz", 0.0, "quater Z coordinate"),
            InputPort<double>("w", 0.0, "quater W coordinate"),
        });
    }

    bool setRequest(typename Request::SharedPtr &request) override
    {
        if (!getInput<double>("x", request->target_pose.position.x) ||
            !getInput<double>("y", request->target_pose.position.y) ||
            !getInput<double>("z", request->target_pose.position.z) ||
            !getInput<double>("wx", request->target_pose.orientation.x) ||
            !getInput<double>("wy", request->target_pose.orientation.y) ||
            !getInput<double>("wz", request->target_pose.orientation.z) ||
            !getInput<double>("w", request->target_pose.orientation.w))
        {
            RCLCPP_ERROR(logger(), "Failed to get input ports");
            return false;
        }
        return true;
    }
    NodeStatus onResponseReceived(const typename Response::SharedPtr &response) override
    {
        if (response->success)
        {
            RCLCPP_INFO(logger(), "MoveTo action succeeded");
            return NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(logger(), "MoveTo action failed");
            return NodeStatus::FAILURE;
        }
    }

    virtual NodeStatus onFailure(ServiceNodeErrorCode /*error*/) override
    {
        RCLCPP_ERROR(logger(), "Service call failed");
        return NodeStatus::FAILURE;
    }
};