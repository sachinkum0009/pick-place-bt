#include <behaviortree_cpp/action_node.h>
#include <chrono>
#include <thread>

namespace BT
{

class WaitNode : public SyncActionNode
{
public:
    WaitNode(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config)
    {
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<int>("delay_msec", 1000, "Delay duration in milliseconds")
        };
    }

    NodeStatus tick() override
    {
        int delay_msec = 1000;  // Default 1 second
        
        if (!getInput<int>("delay_msec", delay_msec))
        {
            // If input port is not provided, use default value
            delay_msec = 1000;
        }

        // Execute the delay
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_msec));
        
        return NodeStatus::SUCCESS;
    }
};

}  // namespace BT
