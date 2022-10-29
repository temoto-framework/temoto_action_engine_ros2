#include "rclcpp/rclcpp.hpp"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/umrf_json_converter.h"
#include "temoto_action_engine/umrf_graph_fs.h"
#include "temoto_action_engine_ros2/msg/broadcast_start_umrf_graph.hpp"

#include <boost/program_options.hpp>
#include <exception>

using std::placeholders::_1;
using namespace temoto_action_engine_ros2::msg;
namespace po = boost::program_options;

int main(int argc, char** argv)
{
	/*
	 * Parse the command line arguments
	 */
	po::options_description description;
	po::variables_map vm;
	description.add_options()
		("target", po::value<std::string>(), "Required. Indicates which Action Engine the graph is sent to.")
		("umrf-graph-path", po::value<std::string>(), "Required. Path to the UMRF graph JSON file.");

	po::store(po::parse_command_line(argc, argv, description), vm);
	po::notify(vm);

	// Parse the target Action Engine
	if (!vm.count("target"))
	{
		std::stringstream ss;
		ss << "Missing the target Action Engine\n" << description;
		throw std::runtime_error(ss.str());
	}
	std::string target = vm["target"].as<std::string>();

	// Parse the path to the UMRF graph JSON file
	if (!vm.count("umrf-graph-path"))
	{
		std::stringstream ss;
		ss << "Missing path to the UMRF graph JSON file\n" << description;
		throw std::runtime_error(ss.str());
	}
	std::string umrf_graph_json_file = vm["umrf-graph-path"].as<std::string>();

	std::cout << " - TARGETED ACTION ENGINE: " << target << std::endl;
    std::cout << " - UMRF_GRAPH_PATH       : " << umrf_graph_json_file << std::endl;

	/*
	 * Initialize ROS
	 */
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("umrf_graph_publisher");

	rclcpp::Publisher<BroadcastStartUmrfGraph>::SharedPtr umrf_graph_pub;
	umrf_graph_pub =
      node->create_publisher<BroadcastStartUmrfGraph>("/broadcast_start_umrf_graph", 10);

	/*
	 * Read the UMRF Graph JSON
	 */
	std::string umrf_graph_json_str = temoto_action_engine::readFromFile(umrf_graph_json_file);
	UmrfGraph umrf_graph = umrf_json_converter::fromUmrfGraphJsonStr(umrf_graph_json_str);
	std::cout << "- UMRF GRAPH JSON:\n" << umrf_graph_json_str << std::endl;

	/*
	 * Create UMRF Graph ROS message
	 */
	auto ujg_msg = BroadcastStartUmrfGraph();
	ujg_msg.umrf_graph_name = umrf_graph.getName();
	ujg_msg.name_match_required = 1;
	ujg_msg.targets.push_back(target);
	ujg_msg.umrf_graph_json = umrf_graph_json_str;

	/*
	 * Publish the UMRF JSON graph
	 */
	std::cout << "Publishing the UMRF Graph message ...";
	umrf_graph_pub->publish(ujg_msg);
	std::cout << "UMRF Graph message published, shutting down." << std::endl;
	return 0;
}
