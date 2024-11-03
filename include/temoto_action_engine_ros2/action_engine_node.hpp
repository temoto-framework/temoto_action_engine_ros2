/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2019 TeMoto Telerobotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef TEMOTO_ACTION_ENGINE__ACTION_ENGINE_ROS2_H
#define TEMOTO_ACTION_ENGINE__ACTION_ENGINE_ROS2_H

#include "rclcpp/rclcpp.hpp"
#include <temoto_action_engine/action_engine.h>
#include <temoto_action_engine/arg_parser.h>
#include "temoto_action_engine_ros2/msg/broadcast_start_umrf_graph.hpp"
#include "temoto_action_engine_ros2/msg/broadcast_stop_umrf_graph.hpp"

#include <memory>

using namespace temoto_action_engine_ros2::msg;

class ActionEngineNode : public rclcpp::Node
{
public:
  ActionEngineNode(int argc, char** argv);

private:
  void startUmrfGraphCb(const BroadcastStartUmrfGraph::SharedPtr msg);

  void stopUmrfGraphCb(const BroadcastStopUmrfGraph::SharedPtr msg);

  bool containsWakeWord(const std::vector<std::string>& wake_words_in) const;

  std::unique_ptr<ActionEngine> ae_;
  action_engine::ArgParser arg_parser_;
  std::vector<std::string> wake_words_;
  std::vector<std::string> action_paths_;
  rclcpp::Subscription<BroadcastStartUmrfGraph>::SharedPtr start_umrf_graph_sub_;
  rclcpp::Subscription<BroadcastStopUmrfGraph>::SharedPtr stop_umrf_graph_sub_;
  std::mutex start_umrf_graph_mutex_;
  std::mutex stop_umrf_graph_mutex_;
};

#endif
