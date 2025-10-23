#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>  // To use radians conversion
#include <cstring>
#include <geometry_msgs/msg/quaternion.hpp>
#include <iostream>
#include <memory>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "utile.hpp"

using namespace std::chrono_literals;

void normalizeQuaternion(geometry_msgs::msg::Quaternion& quat) {
  double norm = std::sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w);
  if (norm > 0.0) {
    quat.x /= norm;
    quat.y /= norm;
    quat.z /= norm;
    quat.w /= norm;
  }
}

class Publisher : public rclcpp::Node {
 public:
  Publisher() : Node("zerolab_tf_broadcaster") {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_passthrough", 10);
    joint_state.name = pnd_hand_names_;
    finger_pos.resize(12, 0);

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = "world";
    transform_stamped.transform.translation.x = 0;
    transform_stamped.transform.translation.y = 0;
    transform_stamped.transform.translation.z = 0;
    transform_stamped.transform.rotation.x = 0;
    transform_stamped.transform.rotation.y = 0;
    transform_stamped.transform.rotation.z = 0;
    transform_stamped.transform.rotation.w = 1;
    for (auto& name : body_names) {
      transform_stamped.child_frame_id = name;
      data_.push_back(transform_stamped);
    }
  }

  void udpToRos() {
    int udp_port = 18000;
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
      return;
    }

    struct sockaddr_in servaddr;
    std::memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;  // inet_addr(udp_ip.c_str());
    servaddr.sin_port = htons(udp_port);

    if (bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP port");
      close(sockfd);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "UDP connection established");

    uint8_t buffer[992];  // Adjust buffer size based on expected packet size
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);

    while (rclcpp::ok()) {
      ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cliaddr, &len);
      if (n > 0) {
        // RCLCPP_INFO(this->get_logger(), "Received UDP data");

        // Ensure the correct number of values are received
        if (n != 992) {
          RCLCPP_ERROR(this->get_logger(), "Incomplete data received, data length:%zd", n);
          continue;
        }

        // Decode data
        std::vector<uint8_t> data(buffer, buffer + 764);
        std::vector<float> decodedData = decodeFloatByteArray(data);

        // Fill root position
        // float x = decodedData[0];
        // float y = decodedData[1];
        // float z = decodedData[2];
        // RCLCPP_INFO(this->get_logger(), "Received root position: [%f, %f, %f]", x, y, z);

        // Fill 47 joint eulers
        for (size_t i = 0; i < body_size; ++i) {
          // left-handed coordinate system to right-handed coordinate system
          data_[i].transform.rotation.x = -decodedData[3 + i * 4 + 2];
          data_[i].transform.rotation.y = decodedData[3 + i * 4 + 0];
          data_[i].transform.rotation.z = -decodedData[3 + i * 4 + 1];
          data_[i].transform.rotation.w = decodedData[3 + i * 4 + 3];
        }

        // ROS_INFO("quat assign completed!");

        std::vector<uint8_t> dataUint16(buffer + 764, buffer + 788);
        std::vector<uint16_t> decodedDataUint16 = bytesToUInt16Vector(dataUint16);

        // ROS_INFO("decodedDataUint16 transfer completed!");

        for (int i = 0; i < 6; ++i) {
          int lfinger = decodedDataUint16[i];
          // std::cout << lfinger << std::endl;
        }
        // 左手拇指、食指、中指、无名指、小指的弯曲值、拇指的旋转值
        finger_pos[0] = 1000 - decodedDataUint16[4];
        finger_pos[1] = 1000 - decodedDataUint16[3];
        finger_pos[2] = 1000 - decodedDataUint16[2];
        finger_pos[3] = 1000 - decodedDataUint16[1];
        finger_pos[4] = 1000 - decodedDataUint16[0];
        finger_pos[5] = 1000 - decodedDataUint16[5];

        for (int i = 0; i < 6; ++i) {
          int rfinger = decodedDataUint16[i + 6];
          // std::cout << rfinger << std::endl;
        }
        // 右手拇指、食指、中指、无名指、小指的弯曲值、拇指的旋转值
        finger_pos[6] = 1000 - decodedDataUint16[4 + 6];
        finger_pos[7] = 1000 - decodedDataUint16[3 + 6];
        finger_pos[8] = 1000 - decodedDataUint16[2 + 6];
        finger_pos[9] = 1000 - decodedDataUint16[1 + 6];
        finger_pos[10] = 1000 - decodedDataUint16[0 + 6];
        finger_pos[11] = 1000 - decodedDataUint16[5 + 6];
        finger_scale();

        std::vector<uint8_t> posBuf(buffer + 788, buffer + 992);
        std::vector<float> decodedPos = decodeFloatByteArray(posBuf);
        for (size_t i = 0; i < body_size; ++i) {
          // left-handed coordinate system to right-handed coordinate system
          data_[i].transform.translation.x = decodedPos[i * 3 + 2];
          data_[i].transform.translation.y = -decodedPos[i * 3 + 0];
          data_[i].transform.translation.z = decodedPos[i * 3 + 1];
        }

        for (size_t idx = 0; idx < data_.size(); ++idx) {
          data_[idx].header.stamp = this->now();
        }
        tf_broadcaster_->sendTransform(std::move(data_));
        pub_jointstates();
      }
    }

    close(sockfd);
  }

  void pub_jointstates() {
    joint_state.header.stamp = this->now();
    joint_state.position = finger_pos;
    joint_state_publisher_->publish(joint_state);
  }
  void finger_scale() {
    for (size_t i = 0; i < finger_pos.size(); ++i) {
      if (i == 4 || i == 5) {
        if (finger_pos[i] < 300) {
          finger_pos[i] = 0;
        } else if (finger_pos[i] > 700) {
          finger_pos[i] = 1000;
        } else {
          finger_pos[i] = 2.5 * (finger_pos[i] - 300);
        }
      }
      if (finger_pos[i] < 200) {
        finger_pos[i] = 0;
      } else if (finger_pos[i] > 800) {
        finger_pos[i] = 1000;
      } else {
        finger_pos[i] = (5.0 / 3.0) * (finger_pos[i] - 200);
      }
    }
  }

 private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<geometry_msgs::msg::TransformStamped> data_;
  std::vector<std::string> body_names = {"Head",     "Spine",         "LeftShoulder", "LeftArm",      "LeftForeArm",
                                         "LeftHand", "RightShoulder", "RightArm",     "RightForeArm", "RightHand",
                                         "Hips",     "LeftUpLeg",     "LeftLeg",      "LeftFoot",     "RightUpLeg",
                                         "RightLeg", "RightFoot"};
  size_t body_size = body_names.size();

  // joint state publisher
  std::vector<std::string> pnd_hand_names_{
      "dof_pos/hand_pinky_Left",  "dof_pos/hand_ring_Left",     "dof_pos/hand_middle_Left",
      "dof_pos/hand_index_Left",  "dof_pos/hand_thumb_1_Left",  "dof_pos/hand_thumb_2_Left",
      "dof_pos/hand_pinky_Right", "dof_pos/hand_ring_Right",    "dof_pos/hand_middle_Right",
      "dof_pos/hand_index_Right", "dof_pos/hand_thumb_1_Right", "dof_pos/hand_thumb_2_Right"};
  std::vector<double> finger_pos;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  sensor_msgs::msg::JointState joint_state;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  Publisher publisher;
  publisher.udpToRos();
  rclcpp::shutdown();
  return 0;
}
