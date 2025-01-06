#include <memory>
#include <string>
#include <sqlite3.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class TestSQLite : public rclcpp::Node
{
  public:
  TestSQLite()
  : Node("test_package")
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "sqlite_topic", 10, std::bind(&TestSQLite::topic_callback, this, _1)
    );
  }
  private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestSQLite>());
  rclcpp::shutdown();
  return 0;
}