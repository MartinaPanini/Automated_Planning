#include <memory>
#include <string>
#include <map>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class CoolDownAction : public plansys2::ActionExecutorClient
{
public:
  CoolDownAction()
  : plansys2::ActionExecutorClient("cool_down", 250ms) // 4Hz
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.02; 
      
      send_feedback(progress_, "Cooling artifact...");
    } else {
      finish(true, 1.0, "Artifact cooled");
      progress_ = 0.0;
    }
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoolDownAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "cool_down"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}