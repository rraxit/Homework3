#include "rclcpp/rclcpp.hpp"
#include "planner_interface/srv/find_path.hpp"
#include <vector>
#include <geometry_msgs/msg/point.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class FindPathClient: public rclcpp::Node
{
  using REQUEST = planner_interface::srv::FindPath::Request::SharedPtr;
  public:
  FindPathClient( char **argv):Node("FindPathClient")
  {
    RCLCPP_INFO(get_logger(), "FindPathClient started");
    client_ = create_client<planner_interface::srv::FindPath>("path_finder");
    auto request = getRequest(argv);
    requestPath(request);
  }

  void requestPath(REQUEST request)
  {

      while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return ;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      //process request
      auto result = client_->async_send_request(request);
      // Wait for the result.
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        processResult(result.get()->path);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to compute path");
      }
  }

  REQUEST getRequest(char **argv)
  {
    
    auto request = std::make_shared<planner_interface::srv::FindPath::Request>();
    request->planner = argv[1];
    request->start = argv[2];
    request->destination = argv[3];
    //TODO: populate request for the server
    return request;
  }

  void processResult(const std::vector<geometry_msgs::msg::Point>& result)
  {
    RCLCPP_INFO(get_logger(), "server responded");
     //TODO show response from server
     for(auto &point: result)
     {
      RCLCPP_INFO(get_logger(), "(%lf, %lf)", point.x, point.y);
     }
   
  }


  private:
    rclcpp::Client<planner_interface::srv::FindPath>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  assert(argc == 4);

  auto clientNode = std::make_shared<FindPathClient>(argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(clientNode);
  rclcpp::shutdown();
  return 0;
}