#include <behaviortree_ros2/bt_service_node.hpp>
#include "pick_place_interface/srv/tag_pose.hpp"

using TagPose = pick_place_interface::srv::TagPose;
using namespace BT;

class TagPoseService : public RosServiceNode<TagPose>
{
public:
    TagPoseService(const std::string &name, const NodeConfig &conf, const RosNodeParams &params)
        : RosServiceNode<TagPose>(name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        PortsList ports = {
            InputPort<uint8_t>("id", 0, "Tag ID"),
            OutputPort<bool>("success", "Success flag"),
            OutputPort<double>("x", "X position"),
            OutputPort<double>("y", "Y position"),
            OutputPort<double>("z", "Z position"),
        };
        return providedBasicPorts(ports);
    }

    bool setRequest(typename Request::SharedPtr &request) override
    {
        if (!getInput<uint8_t>("id", request->id))
        {
            RCLCPP_ERROR(logger(), "Failed to get input port 'id'");
            return false;
        }
        return true;
    }

    NodeStatus onResponseReceived(const typename Response::SharedPtr &response) override
    {
        if (response->success)
        {
            setOutput("success", true);
            setOutput("x", response->pose.position.x);
            setOutput("y", response->pose.position.y);
            setOutput("z", response->pose.position.z);
            RCLCPP_INFO(logger(), "TagPose service call succeeded, pose is %f, %f, %f",
                        response->pose.position.x,
                        response->pose.position.y,
                        response->pose.position.z);
            return NodeStatus::SUCCESS;
        }
        else
        {
            setOutput("success", false);
            RCLCPP_ERROR(logger(), "TagPose service call failed");
            return NodeStatus::FAILURE;
        }
    }

    virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "error is %d", static_cast<int>(error));
        RCLCPP_ERROR(logger(), "Service call failed");
        return NodeStatus::FAILURE;
    }
};