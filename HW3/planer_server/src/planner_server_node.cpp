#include "rclcpp/rclcpp.hpp"
#include "planner_interface/srv/find_path.hpp"
#include <geometry_msgs/msg/point.hpp>
#include "bfs_hw.cpp"

class PathFinder : public rclcpp::Node
{
public:
  PathFinder()
    : Node("path_finder")
  {
    populateGraph();
    // Create the service
    service_ = create_service<planner_interface::srv::FindPath>(
      "path_finder", std::bind(&PathFinder::getPlan, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Ready to find path.");
  }

private:


  void populateGraph()
  {
    //TODO poulate your search graph here...
  }


  void getPlan(const std::shared_ptr<planner_interface::srv::FindPath::Request> request,
               std::shared_ptr<planner_interface::srv::FindPath::Response> response)
  {
    const std::string plannerName = request->planner; // Fixed the declaration
    std::string start_node = request->start;    // Fixed the declaration
    std::string destination = request->destination; // Fixed the declaration

    std::vector<std::string> path1 = bfs(start_node,destination); 
    for (const auto &node : path1)
    {
    
    geometry_msgs::msg::Point p;
    p.x = graph_nodes[node].x;
    p.y = graph_nodes[node].y;
    p.z = graph_nodes[node].z;
    response->path.push_back(p);

    }

    //TODO: Your service logic goes here
    // You can access the request data using request-><field_name>
    // You can fill the response data using response-><field_name>

  }

  rclcpp::Service<planner_interface::srv::FindPath>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PathFinder>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
