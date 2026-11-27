#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mid360_agras_driver/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stdexcept>
#include <string>

namespace {

using AgrasCloud = mid360_agras_driver::msg::CustomMsg;
using LivoxCloud = livox_ros_driver2::msg::CustomMsg;

class CustomMsgBridge : public rclcpp::Node {
 public:
  CustomMsgBridge() : Node("mid360_agras_livox_compat") {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/agras/livox/lidar");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/livox/lidar");
    const auto depth = declare_parameter<int64_t>("qos_depth", 0);
    const auto topics = get_node_topics_interface();
    if (topics->resolve_topic_name(input_topic) == topics->resolve_topic_name(output_topic)) {
      throw std::invalid_argument("input_topic and output_topic must be different");
    }

    if (depth != 0 && depth != 64 && depth != 256) {
      throw std::invalid_argument("qos_depth must be 0, 64 or 256");
    }
    const rclcpp::QoS qos(depth == 0 ? 16 : static_cast<size_t>(depth));
    publisher_ = create_publisher<LivoxCloud>(output_topic, qos);
    const rclcpp::QoS input_qos = depth == 0 ? rclcpp::SensorDataQoS() : qos;
    subscription_ = create_subscription<AgrasCloud>(
        input_topic, input_qos,
        [this](const AgrasCloud& source) { publish(source); });
    if (depth != 0) {
      RCLCPP_INFO(get_logger(), "Livox CustomMsg QoS: input=%zu output=%zu",
                  subscription_->get_actual_qos().depth(), publisher_->get_actual_qos().depth());
    }
  }

 private:
  void publish(const AgrasCloud& source) {
    LivoxCloud output;
    output.header = source.header;
    output.timebase = source.timebase;
    output.point_num = source.point_num;
    output.lidar_id = source.lidar_id;
    output.rsvd = source.rsvd;
    output.points.resize(source.points.size());
    for (std::size_t index = 0; index < source.points.size(); ++index) {
      const auto& input = source.points[index];
      auto& point = output.points[index];
      point.offset_time = input.offset_time;
      point.x = input.x;
      point.y = input.y;
      point.z = input.z;
      point.reflectivity = input.reflectivity;
      point.tag = input.tag;
      point.line = input.line;
    }
    publisher_->publish(output);
  }

  rclcpp::Publisher<LivoxCloud>::SharedPtr publisher_;
  rclcpp::Subscription<AgrasCloud>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomMsgBridge>());
  rclcpp::shutdown();
  return 0;
}
