#include <behaviortree_cpp_v3/bt_factory.h>
#include <iostream>
#include <chrono>
#include <thread>

// Global battery status (for simulation)
bool battery_low = false;

// Custom Action: Navigate
class NavigateAction : public BT::SyncActionNode
{
public:
  NavigateAction(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {}

  // Required static method for BehaviorTree.CPP
  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    if(battery_low)
    {
      std::cout << "Battery is low, cannot navigate normally.\n";
      return BT::NodeStatus::FAILURE;
    }
    std::cout << "Navigating to waypoint...\n";
    // Simulate navigation delay
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "Reached waypoint.\n";
    return BT::NodeStatus::SUCCESS;
  }
};

// Custom Condition: CheckBattery
BT::NodeStatus CheckBattery(BT::TreeNode &)
{
  if(battery_low)
  {
    std::cout << "Battery is low.\n";
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    std::cout << "Battery is OK.\n";
    return BT::NodeStatus::SUCCESS;
  }
}

// Custom Action: GoToCharging
class GoToChargingAction : public BT::SyncActionNode
{
public:
  GoToChargingAction(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {}

  // Required static method for BehaviorTree.CPP
  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    std::cout << "Navigating to charging station...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "Reached charging station.\n";
    return BT::NodeStatus::SUCCESS;
  }
};

// Custom Action: Recharge
class RechargeAction : public BT::SyncActionNode
{
public:
  RechargeAction(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {}

  // Required static method for BehaviorTree.CPP
  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    std::cout << "Recharging battery (30 seconds)...\n";
    std::this_thread::sleep_for(std::chrono::seconds(30));
    battery_low = false;
    std::cout << "Battery recharged.\n";
    return BT::NodeStatus::SUCCESS;
  }
};

int main()
{
  BT::BehaviorTreeFactory factory;

  // Register custom nodes with the factory
  factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery, std::placeholders::_1));
  factory.registerNodeType<NavigateAction>("Navigate");
  factory.registerNodeType<GoToChargingAction>("GoToCharging");
  factory.registerNodeType<RechargeAction>("Recharge");

  // Define a simple behavior tree in XML
  static const char *xml_text = R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <Fallback name="root_fallback">
        <Sequence name="normal_navigation">
          <Condition ID="CheckBattery"/>
          <Navigate/>
        </Sequence>
        <Sequence name="charging_sequence">
          <GoToCharging/>
          <Recharge/>
        </Sequence>
      </Fallback>
    </BehaviorTree>
  </root>
  )";

  // Build the tree
  auto tree = factory.createTreeFromText(xml_text);

  // Simulate a series of navigation tasks
  int task_count = 0;
  while(task_count < 10)
  {
    std::cout << "\n--- Task " << task_count+1 << " ---\n";
    // Simulate battery drain: after 4 tasks, battery becomes low
    if(task_count == 4)
    {
      battery_low = true;
      std::cout << "Simulated: Battery becomes low.\n";
    }
    // Tick the tree until it returns SUCCESS or FAILURE
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while(status == BT::NodeStatus::RUNNING)
    {
      status = tree.tickRoot();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    task_count++;
  }
  return 0;
}

