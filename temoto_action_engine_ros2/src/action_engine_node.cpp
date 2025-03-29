#include <temoto_action_engine/action_engine.h>
#include <temoto_action_engine/arg_parser.h>
#include <temoto_action_engine/umrf_json.h>

#include "temoto_msgs/msg/umrf_graph_start.hpp"
#include "temoto_msgs/msg/umrf_graph_stop.hpp"
#include "temoto_msgs/msg/umrf_graph_pause.hpp"
#include "temoto_msgs/msg/umrf_graph_resume.hpp"
#include "temoto_msgs/msg/umrf_graph_feedback.hpp"

#include "temoto_msgs/srv/umrf_graph_get.hpp"

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <thread>

using std::placeholders::_1;
using std::placeholders::_2;

using namespace temoto_msgs::msg;
using namespace temoto_msgs::srv;

class ActionEngineNode : public rclcpp::Node
{
public:

  ActionEngineNode(int argc, char** argv)
  : Node("action_engine_node")
  , arg_parser_(argc, argv)
  {
    /*
     * Get the wake words and actions path
     */
    wake_words_    = arg_parser_.getWakeWords();
    action_paths_  = arg_parser_.getActionPaths();
    indexing_rate_ = arg_parser_.getIndexingRate();

    std::cout << " Initializing the Action Engine" << std::endl;
    std::cout << " - ACTOR NAME   : " << wake_words_.at(0) << std::endl;
    std::cout << " - ACTIONS PATH : " << action_paths_.at(0) << std::endl;

    ae_ = std::make_unique<ActionEngine>(wake_words_.at(0), indexing_rate_);

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

    if (indexing_rate_ == 0 && successful_paths == 0)
    {
      throw CREATE_TEMOTO_ERROR("None of the indicated directories contained TeMoto actions, exiting.");
    }

    /*
     * Start the subscribers and services
     */
    umrf_graph_start_sub_ = this->create_subscription<UmrfGraphStart>(
      "/umrf_graph_start", 1, std::bind(&ActionEngineNode::UmrfGraphStartCb, this, _1));

    umrf_graph_stop_sub_ = this->create_subscription<UmrfGraphStop>(
      "/umrf_graph_stop", 1, std::bind(&ActionEngineNode::UmrfGraphStopCb, this, _1));

    umrf_graph_pause_sub_ = this->create_subscription<UmrfGraphPause>(
      "/umrf_pause_pause", 1, std::bind(&ActionEngineNode::UmrfGraphPauseCb, this, _1));

    umrf_graph_resume_sub_ = this->create_subscription<UmrfGraphResume>(
      "/umrf_graph_resume", 1, std::bind(&ActionEngineNode::UmrfGraphResumeCb, this, _1));

    umrf_graph_feedback_pub = this->create_publisher<UmrfGraphFeedback>(
      "/umrf_graph_feedback", 10);

    umrf_graph_get_srv_ = this->create_service<UmrfGraphGet>(
      "umrf_graph_get", std::bind(&ActionEngineNode::umrfGraphGetCb, this, _1, _2));

    startFeedbackLoop();

    RCLCPP_INFO(this->get_logger(), "The Action Engine is initialized.");
  }

  ~ActionEngineNode()
  {
    feedback_reader_thread_stop_ = true;
    feedback_reader_thread_.join();

    // TODO: Create a manager thread, which joins the graph-wait threads continuously
    for (auto& t : wait_thread_pool_)
    {
      t.join();
    }
  }

private:

  void startFeedbackLoop()
  {
    feedback_reader_thread_stop_ = false;
    feedback_reader_thread_ = std::thread([&]
    {
      while (feedback_reader_thread_stop_ == false)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::vector<std::string> feedback{ae_->readFeedbackBuffer()};

        if (feedback.empty())
        {
          continue;
        }

        UmrfGraphFeedback feedback_msg;
        feedback_msg.actor = wake_words_.at(0);
        feedback_msg.history = feedback;

        umrf_graph_feedback_pub->publish(feedback_msg);
      }
    });
  }

  void UmrfGraphStartCb(const UmrfGraphStart::SharedPtr msg)
  try
  {
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
    std::string graph_name;

    if (msg->umrf_graph_json.empty())
    {
      graph_name = msg->umrf_graph_name;
      ae_->executeUmrfGraph(graph_name);
    }
    else
    {
      UmrfGraph umrf_graph = umrf_json::fromUmrfGraphJsonStr(msg->umrf_graph_json);
      graph_name = umrf_graph.getName();
      ae_->executeUmrfGraphA(umrf_graph, "on_true", bool(msg->name_match_required));
    }

    wait_thread_pool_.push_back(std::move(std::thread([&, gn = graph_name]
    {
      ae_->waitForGraph(gn);
      RCLCPP_INFO(this->get_logger(), "Graph '%s' finished.", gn.c_str());
    })));

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

  void UmrfGraphStopCb(const UmrfGraphStop::SharedPtr msg)
  {
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

  void UmrfGraphPauseCb(const UmrfGraphPause::SharedPtr msg)
  {
  }

  void UmrfGraphResumeCb(const UmrfGraphResume::SharedPtr msg)
  {
  }

  void umrfGraphGetCb(const UmrfGraphGet::Request::SharedPtr, const UmrfGraphGet::Response::SharedPtr response)
  {
    response->umrf_jsons = ae_->getUmrfJsons();
    response->graph_jsons_indexed = ae_->getGraphJsonsIndexed();
    response->graph_jsons_running = ae_->getGraphJsonsRunning();
  }

  bool containsWakeWord(const std::vector<std::string>& wake_words_in) const
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

  std::unique_ptr<ActionEngine> ae_;
  action_engine::ArgParser arg_parser_;
  std::vector<std::string> wake_words_;
  std::vector<std::string> action_paths_;
  unsigned int indexing_rate_;

  rclcpp::Subscription<UmrfGraphStart>::SharedPtr  umrf_graph_start_sub_;
  rclcpp::Subscription<UmrfGraphStop>::SharedPtr   umrf_graph_stop_sub_;
  rclcpp::Subscription<UmrfGraphPause>::SharedPtr  umrf_graph_pause_sub_;
  rclcpp::Subscription<UmrfGraphResume>::SharedPtr umrf_graph_resume_sub_;
  rclcpp::Publisher<UmrfGraphFeedback>::SharedPtr  umrf_graph_feedback_pub;

  std::thread feedback_reader_thread_;
  bool feedback_reader_thread_stop_;

  rclcpp::Service<UmrfGraphGet>::SharedPtr umrf_graph_get_srv_;

  std::vector<std::thread> wait_thread_pool_;
};

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
