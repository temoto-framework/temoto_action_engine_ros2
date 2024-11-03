#include "temoto_action_engine_ros2/action_engine_node.hpp"
#include <temoto_action_engine/umrf_json.h>

using std::placeholders::_1;

ActionEngineNode::ActionEngineNode(int argc, char** argv)
: Node("action_engine_node")
, arg_parser_(argc, argv)
{
  /*
   * Get the wake words and actions path
   */
  wake_words_ = arg_parser_.getWakeWords();
  action_paths_ = arg_parser_.getActionPaths();

  std::cout << " Initializing the Action Engine" << std::endl;
  std::cout << " - ACTOR NAME   : " << wake_words_.at(0) << std::endl;
  std::cout << " - ACTIONS PATH : " << action_paths_.at(0) << std::endl;

  ae_ = std::make_unique<ActionEngine>(wake_words_.at(0));

  /*
   * Check if the paths contain any TeMoto actions
   */
  int successful_paths = 0;
  for (const auto& ap : action_paths_)
  {
    try
    {
      if (ae_->addActionsPath(ap))
      {
        successful_paths++;
      }
    }
    catch(const std::exception& e)
    {
      TEMOTO_PRINT(e.what());
    }
  }

  if (successful_paths == 0)
  {
    throw CREATE_TEMOTO_ERROR("None of the indicated directories contained TeMoto actions, exiting.");
  }

  /*
   * Start the subscribers and services
   */
  start_umrf_graph_sub_ = this->create_subscription<BroadcastStartUmrfGraph>(
    "broadcast_start_umrf_graph", 1, std::bind(&ActionEngineNode::startUmrfGraphCb, this, _1));

  stop_umrf_graph_sub_ = this->create_subscription<BroadcastStopUmrfGraph>(
    "broadcast_stop_umrf_graph", 1, std::bind(&ActionEngineNode::stopUmrfGraphCb, this, _1));

  RCLCPP_INFO(this->get_logger(), "The Action Engine is initialized.");
}

void ActionEngineNode::startUmrfGraphCb(const BroadcastStartUmrfGraph::SharedPtr msg)
try
{
  std::lock_guard<std::mutex> lock(start_umrf_graph_mutex_);
  RCLCPP_INFO(this->get_logger(), "Received request to start UMRF graph: %s", msg->umrf_graph_name.c_str());

  // If the wake word was not found then return
  if (!containsWakeWord(msg->targets))
  {
    RCLCPP_INFO(this->get_logger(), "The UMRF graph message was not targeted at this Action Engine.");
    return;
  }

  /*
   * Check wether it's a diff request or new graph request
   */
  if (msg->umrf_graph_json.empty())
  {
    ae_->executeUmrfGraph(msg->umrf_graph_name);
  }
  else
  {
    UmrfGraph umrf_graph = umrf_json::fromUmrfGraphJsonStr(msg->umrf_graph_json);
    ae_->executeUmrfGraphA(umrf_graph, "on_true", bool(msg->name_match_required));
  }
  
  // else if (!msg->umrf_graph_diffs.empty())
  // {
  //   /*
  //    * Modify an existing umrf graph according to the diff specifiers
  //    */
  //   try
  //   {
  //     UmrfGraphDiffs umrf_graph_diffs;
  //     for(const auto& umrf_graph_diff_msg : msg->umrf_graph_diffs)
  //     {
  //       UmrfNode umrf_diff = umrf_json_converter::fromUmrfJsonStr(umrf_graph_diff_msg.umrf_json);
  //       umrf_graph_diffs.emplace_back(umrf_graph_diff_msg.operation, umrf_diff);
  //     }

  //     ae_.modifyGraph(msg->umrf_graph_name, umrf_graph_diffs);
  //   }
  //   catch(const std::exception& e)
  //   {
  //     RCLCPP_INFO(this->get_logger(), std::string(e.what()).c_str());
  //   }
  // }
}
catch(const std::exception& e)
{
  RCLCPP_INFO(this->get_logger(), std::string(e.what()).c_str());
}

void ActionEngineNode::stopUmrfGraphCb(const BroadcastStopUmrfGraph::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(stop_umrf_graph_mutex_);
  RCLCPP_INFO(this->get_logger(), "Received request to stop UMRF graph: %s", msg->umrf_graph_name.c_str());

  // If the wake word was not found then return
  if (!containsWakeWord(msg->targets))
  {
    RCLCPP_INFO(this->get_logger(), "The stop message was not targeted at this Action Engine.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Stopping UMRF graph '%s' ...", msg->umrf_graph_name.c_str());
  try
  {
    ae_->stopUmrfGraph(msg->umrf_graph_name);
    RCLCPP_INFO(this->get_logger(), "UMRF graph '%s' stopped.", msg->umrf_graph_name.c_str());
  }
  catch(const std::exception& e)
  {
    RCLCPP_INFO(this->get_logger(), std::string(e.what()).c_str());
  }
}

bool ActionEngineNode::containsWakeWord(const std::vector<std::string>& wake_words_in) const
{
  for (const auto& target : wake_words_in)
  {
    for (const auto& ww : wake_words_)
    {
      if (ww == target)
      {
        return true;
      }
    }
  }
  return false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<ActionEngineNode> ae_node;

  /*
   * Parse the commandline arguments
   */
  try
  {
    ae_node = std::make_shared<ActionEngineNode>(argc, argv);
  }
  catch(const std::runtime_error& e)
  {
    std::cout << e.what() << std::endl;
    return 1;
  }

  /*
   * Start the node
   */
  rclcpp::spin(ae_node);
  rclcpp::shutdown();
  return 0;
}
