
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rslidar_driver/rsdriver.hpp"
#include "rslidar_pointcloud/convert.hpp"

int main(int argc, char* argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  
  rclcpp::NodeOptions options(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto driver = std::make_shared<rslidar_driver::rslidarDriver>(options);
  exec.add_node(driver);
  
  auto converter = std::make_shared<rslidar_pointcloud::Convert>(options);
  exec.add_node(converter);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}