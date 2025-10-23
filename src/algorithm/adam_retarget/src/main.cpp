#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <casadi/casadi.hpp>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

#include "adam_retarget/Retarget.hpp"
#include "adam_retarget/utils.hpp"

double timecost(std::function<void()> func) {
  auto start = std::chrono::high_resolution_clock::now();
  func();
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
  return duration;
}

// #define ZXW_DEBUG
using namespace std::chrono_literals;

class AdamRetarget : public rclcpp::Node {
 public:
  AdamRetarget() : Node("adam_retarget") {
    // Declare and acquire `target_frame` parameter
    base_frame = this->declare_parameter<std::string>("base_frame", "world");
    control_loop_rate = this->declare_parameter<double>("control_loop_rate", 100);
    config_json_path = this->declare_parameter<std::string>("config_json_path", "config.json");
    warm_start_trig_timeout = this->declare_parameter<double>("warm_start_trig_timeout", 0.5);      // seconds
    warm_start_duration = this->declare_parameter<double>("warm_start_duration", 5.0);              // seconds
    warm_start_slowdown_ratio = this->declare_parameter<double>("warm_start_slowdown_ratio", 0.1);  // seconds
    RCLCPP_INFO(this->get_logger(), "Base frame: %s", base_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "Control loop rate: %f", control_loop_rate);
    RCLCPP_INFO(this->get_logger(), "Config json path: %s", config_json_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Warm start duration: %f", warm_start_duration);
    RCLCPP_INFO(this->get_logger(), "Warm start slowdown ratio: %f", warm_start_slowdown_ratio);
    RCLCPP_INFO(this->get_logger(), "Warm start trig timeout: %f", warm_start_trig_timeout);

    // check if the file exists
    {
      std::ifstream file(config_json_path);
      if (!file.good()) {
        RCLCPP_ERROR(this->get_logger(), "Config file %s does not exist", config_json_path.c_str());
        throw std::runtime_error("Config file does not exist");
      }
    }
    retarget_optimization = std::make_shared<RetargetOptimization>(config_json_path, this->get_logger());

    // declare custom parameters
    for (auto &param : retarget_optimization->getDefaultWeights()) {
      double default_value = double(param.second);
      auto param_name = "custom_weight." + param.first;
      auto param_value = this->declare_parameter<double>(param_name, default_value);
      if (param_value != default_value) {
        RCLCPP_INFO(this->get_logger(), "Custom weight %s is set to %f", param_name.c_str(), param_value);
        retarget_optimization->setDefaultWeight(param.first, param_value);
      }
    }

    // print all the parameters
    RCLCPP_INFO(this->get_logger(), "Optimization parameters:");
    for (auto &param : retarget_optimization->getDefaultWeights()) {
      RCLCPP_INFO(this->get_logger(), "%s: %f", param.first.c_str(), double(param.second));
    }

    needed_bone_names = retarget_optimization->boneNames();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Call on_timer function every second
    auto dt = 1.0 / control_loop_rate * 1s;
    timer_ = this->create_wall_timer(dt, std::bind(&AdamRetarget::on_timer, this));

    // joint state publisher
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    joint_state_passthrough_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states_passthrough", 10, std::bind(&AdamRetarget::on_passthrough_msg, this, std::placeholders::_1));
    // warm start service
    warm_start_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/warm_start", std::bind(&AdamRetarget::on_warm_start, this, std::placeholders::_1, std::placeholders::_2,
                                  std::placeholders::_3));
    // reset service
    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/reset",
        std::bind(&AdamRetarget::on_reset, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
#ifdef ZXW_DEBUG
    log_file.open("/pnd_workspace/log.txt");
#endif
  }

 private:
  void on_warm_start(const std::shared_ptr<rmw_request_id_t> request_header,
                     const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // warm start service
    (void)request_header;
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Warm start triggered by service");
    warm_start_service_triggered = true;
    response->success = true;
    response->message = "Warm start triggered";
  }

  void on_reset(const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // reset service
    (void)request_header;
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Reset triggered by service");
    reset_service_triggered = true;
    response->success = true;
    response->message = "Reset triggered";
  }

  void pub_jointstates() {
    // publish joint state
    auto labels = this->retarget_optimization->getLabels();
    // exclude the joints that are in the passthrough message
    if (last_passthrough_msg_.name.size() > 0) {
      for (size_t i = 0; i < last_passthrough_msg_.name.size(); i++) {
        auto it = std::find(labels.begin(), labels.end(), last_passthrough_msg_.name[i]);
        if (it != labels.end()) {
          // found
          labels.erase(it);
        }
      }
    }
    auto result = this->retarget_optimization->getResult(labels);

    // extend the result with the passthrough message
    if (last_passthrough_msg_.name.size() > 0) {
      RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Passthrough joints: " << last_passthrough_msg_.name);
      labels.insert(labels.end(), last_passthrough_msg_.name.begin(), last_passthrough_msg_.name.end());
      result.insert(result.end(), last_passthrough_msg_.position.begin(), last_passthrough_msg_.position.end());
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->now();
    joint_state.name = labels;
    joint_state.position = result;
    joint_state_publisher_->publish(joint_state);
  }

  bool check_warm_start() {
    if (warm_start_service_triggered) {
      warm_start = true;
      warm_start_service_triggered = false;
      warm_start_timestamp = this->now();
      RCLCPP_INFO_STREAM(this->get_logger(), "Warm start triggered by service");
    }

    // check if the warm start is needed
    // check if two time stamp are the same source
    if (last_tf2_time_stamp_.get_clock_type() != current_tf2_time_stamp_.get_clock_type()) {
      warm_start = true;
      warm_start_timestamp = this->now();
      RCLCPP_INFO_STREAM(this->get_logger(), "First tf received, warm start triggered");
      last_tf2_time_stamp_ = current_tf2_time_stamp_;
      return warm_start;
    }

    if ((current_tf2_time_stamp_ - last_tf2_time_stamp_).seconds() > warm_start_trig_timeout) {
      // warm start is needed
      if (!warm_start)
        RCLCPP_INFO_STREAM(this->get_logger(), "Data timeout for "
                                                   << (current_tf2_time_stamp_ - last_tf2_time_stamp_).seconds()
                                                   << " seconds, warm start triggered for safety");
      warm_start = true;
      warm_start_timestamp = this->now();
      return warm_start;
    }

    if (warm_start) {
      // check if the warm start duration is over
      if ((this->now() - warm_start_timestamp).seconds() > warm_start_duration) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Warm start duration is over, warm start disabled");
        warm_start = false;
        return warm_start;
      }
    }

    return warm_start;
  }

  void on_passthrough_msg(const sensor_msgs::msg::JointState::SharedPtr msg) { last_passthrough_msg_ = *msg; }

  void on_timer() {
    // Get the all transforms available
    casadi::DMDict bonemap;

    std::vector<std::string> missing_bone_names;
    // walk through all the frames and get the transform
    for (auto &frame : needed_bone_names) {
      // check if the transform is available
      std::string cant_transform_err;
      if (!tf_buffer_->canTransform(this->base_frame, frame, tf2::TimePointZero, &cant_transform_err)) {
        missing_bone_names.push_back(frame);
        continue;
        ;
      }

      try {
        auto transform_msg = tf_buffer_->lookupTransform(this->base_frame, frame, tf2::TimePointZero);
        // update the time stamp of the tf2 buffer
        current_tf2_time_stamp_ = transform_msg.header.stamp;
        bonemap[frame] = transform(transform_msg.transform.translation.x, transform_msg.transform.translation.y,
                                   transform_msg.transform.translation.z, transform_msg.transform.rotation.x,
                                   transform_msg.transform.rotation.y, transform_msg.transform.rotation.z,
                                   transform_msg.transform.rotation.w);
      } catch (const std::exception &e) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Failed to get transform from %s to %s",
                             this->base_frame.c_str(), frame.c_str());
      }
    }
    if (bonemap.size() != needed_bone_names.size()) {
      RCLCPP_INFO_STREAM_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Missing bones: " << missing_bone_names.size() << " / " << needed_bone_names.size() << missing_bone_names);
      pub_jointstates();
      return;
    }
    check_warm_start();

    // record solve time cost
    auto start = std::chrono::high_resolution_clock::now();

    casadi::DMDict retarget_params;
    auto param_time_cost =
        timecost([&]() { retarget_params = this->retarget_optimization->calculate_params(bonemap); });

    auto solve_time_cost = timecost([&]() {
      if (reset_service_triggered) {
        this->retarget_optimization->solve(retarget_params, this->retarget_optimization->getLastq0(), 100.0, 1.0);
        reset_service_triggered = false;
      } else {
        this->retarget_optimization->solve(retarget_params, this->retarget_optimization->getLastq0(),
                                           1.0 / this->control_loop_rate, warm_start ? warm_start_slowdown_ratio : 1.0);
      }
    });
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    update_solve_time(duration);

    // print time cost keep 3 decimal
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Retarget time cost: " << std::fixed << std::setprecision(3) << duration << " ms, "
                                                       << "param time cost: " << std::fixed << std::setprecision(3)
                                                       << param_time_cost << " ms, "
                                                       << "solve time cost: " << std::fixed << std::setprecision(3)
                                                       << solve_time_cost << " ms, "
                                                       << "average solve time: " << std::fixed << std::setprecision(3)
                                                       << avr_solve_time << " ms");

    // backup tf2 time stamp
    last_tf2_time_stamp_ = current_tf2_time_stamp_;

    // publish joint states
    pub_jointstates();

#ifdef ZXW_DEBUG

    // write bone map to log file
    log_file << "New Frame" << std::endl;
    log_file << "Bone Map" << std::endl;
    log_file << "{" << std::endl;
    for (auto &bone : bonemap) {
      log_file << bone.first << std::endl;
      log_file << bone.second << std::endl;
    }
    log_file << "}" << std::endl;
    log_file << "Retarget Params" << std::endl;
    log_file << "{" << std::endl;
    for (auto &param : retarget_params) {
      log_file << param.first << std::endl;
      log_file << param.second << std::endl;
    }
    log_file << "}" << std::endl;
    log_file << "Result" << std::endl;
    log_file << "{" << std::endl;
    for (int i = 0; i < labels.size(); i++) {
      log_file << labels[i] << " : " << result[i] << std::endl;
    }
    log_file << "}" << std::endl;
#endif
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string base_frame;
  double control_loop_rate;
  std::string config_json_path;

  // time stamp for the last time tf2 buffer was updated
  rclcpp::Time last_tf2_time_stamp_, current_tf2_time_stamp_;

  bool warm_start = true;
  double warm_start_trig_timeout = 0.5;     // seconds
  double warm_start_duration = 5.0;         // seconds
  double warm_start_slowdown_ratio = 0.05;  // seconds

  rclcpp::Time warm_start_timestamp;  // seconds

  // trigger service for warm start
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr warm_start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  bool warm_start_service_triggered = false;
  bool reset_service_triggered = false;
  std::vector<std::string> needed_bone_names;

  std::shared_ptr<RetargetOptimization> retarget_optimization;

  std::ofstream log_file;

  // joint state publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_passthrough_subscription_;
  sensor_msgs::msg::JointState last_passthrough_msg_;

  double avr_solve_time = 0.0;
  size_t solve_count = 0;
  constexpr static size_t max_solve_count = 100000;
  void update_solve_time(double time) {
    if (solve_count < max_solve_count) solve_count++;

    avr_solve_time = (avr_solve_time * (solve_count - 1) + time) / solve_count;
  }
};

int main(int argc, char *argv[]) {
  // Process ros2-related command-line arguments and initialize ros2 for this process
  rclcpp::init(argc, argv);
  // Create a ros2 node, which owns one or more ros2 interfaces
  auto node = std::make_shared<AdamRetarget>();

  // Spin the node, i.e. start processing messages and callbacks on ros2 interfaces
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}