#include "noitom_mocap/noitom_data_handler.h"

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <iostream>
#include <vector>

#include "MocapApi.h"

namespace pnd {
namespace noitom_mocap {

#define VERIFY(r)                                                    \
  if (auto r_ = (r); r_ != Error_None) {                             \
    std::cout << "\'" << #r "\' return Error : " << r_ << std::endl; \
    throw std::runtime_error("");                                    \
  }

using namespace MocapApi;

void printJointPosture(MCPJointHandle_t jointHandle) {
  IMCPJoint* mcpJoint = nullptr;
  VERIFY(MCPGetGenericInterface(IMCPJoint_Version, reinterpret_cast<void**>(&mcpJoint)));
  const char* szJointName = nullptr;
  VERIFY(mcpJoint->GetJointName(&szJointName, jointHandle));
  // std::cout << "\t" << szJointName << ":";
  float qx, qy, qz, qw;
  // VERIFY(mcpJoint->GetJointLocalRotation(&qx, &qy, &qz, &qw, jointHandle));
  VERIFY(mcpJoint->GetJointGlobalRotation(&qx, &qy, &qz, &qw, jointHandle));
  // std::cout << "(" << qx << "," << qy << "," << qz << "," << qw << ") ";
  float px, py, pz;
  // VERIFY(mcpJoint->GetJointLocalPosition(&px, &py, &pz, jointHandle));
  VERIFY(mcpJoint->GetJointGlobalPosition(&px, &py, &pz, jointHandle));
  // std::cout << "(" << px << "," << py << "," << pz << ")" << std::endl;
  uint32_t unSizeOfJointHandle = 0;
  VERIFY(mcpJoint->GetJointChild(nullptr, &unSizeOfJointHandle, jointHandle));

  if (DataHandler::getInstance().joint_index_.find(szJointName) != DataHandler::getInstance().joint_index_.end()) {
    int idx = DataHandler::getInstance().joint_index_[szJointName];
    DataHandler::getInstance().data_[idx].transform.translation.x = px / 100.0;
    DataHandler::getInstance().data_[idx].transform.translation.y = py / 100.0;
    DataHandler::getInstance().data_[idx].transform.translation.z = pz / 100.0;
    DataHandler::getInstance().data_[idx].transform.rotation.x = qx;
    DataHandler::getInstance().data_[idx].transform.rotation.y = qy;
    DataHandler::getInstance().data_[idx].transform.rotation.z = qz;
    DataHandler::getInstance().data_[idx].transform.rotation.w = qw;

    // tf2::Quaternion q(qx, qy, qz, qw);
    // q.setRPY(M_PI / 2, 0.0, 0.0);  // 绕X轴旋转90度

    // DataHandler::getInstance().data_[idx].transform.translation.x = px / 100.0;
    // DataHandler::getInstance().data_[idx].transform.translation.y = py / 100.0;
    // DataHandler::getInstance().data_[idx].transform.translation.z = pz / 100.0;
    // DataHandler::getInstance().data_[idx].transform.rotation.x = q.x();
    // DataHandler::getInstance().data_[idx].transform.rotation.y = q.y();
    // DataHandler::getInstance().data_[idx].transform.rotation.z = q.z();
    // DataHandler::getInstance().data_[idx].transform.rotation.w = q.w();
  } else {
    std::cout << "Joint name not found in map" << std::endl;
  }

  if (unSizeOfJointHandle > 0) {
    std::vector<MCPJointHandle_t> vJointHandles;
    vJointHandles.resize(unSizeOfJointHandle);
    VERIFY(mcpJoint->GetJointChild(vJointHandles.data(), &unSizeOfJointHandle, jointHandle));
    std::for_each(vJointHandles.begin(), vJointHandles.end(),
                  [](MCPJointHandle_t childJointHandle) { printJointPosture(childJointHandle); });
  }
}

void handleAvatarUpdated(const MCPEvent_MotionData_t& motionData) {
  IMCPAvatar* mcpvatar = nullptr;
  VERIFY(MCPGetGenericInterface(IMCPAvatar_Version, reinterpret_cast<void**>(&mcpvatar)));
  const char* szAvatarName = nullptr;
  VERIFY(mcpvatar->GetAvatarName(&szAvatarName, motionData.avatarHandle));
  // std::cout << "Avatar \'" << szAvatarName << "\' updated" << std::endl;

  MCPJointHandle_t rootJointHandle = 0;
  VERIFY(mcpvatar->GetAvatarRootJoint(&rootJointHandle, motionData.avatarHandle));
  printJointPosture(rootJointHandle);
  DataHandler::getInstance().call();
}

void handleEvent(const MCPEvent_t& ev) {
  switch (ev.eventType) {
    case MCPEvent_AvatarUpdated:
      handleAvatarUpdated(ev.eventData.motionData);
      break;
    case MCPEvent_Error:
      std::cout << "Error occur " << ev.eventData.systemError.error << std::endl;
      break;
    default:
      std::cout << "Unhandled event" << std::endl;
      break;
  }
}

int run() {
  std::cout << "MocapApi Version:" << MCPGetMocapApiVersionString() << std::endl;
  IMCPSettings* mcpSettings = nullptr;
  VERIFY(MocapApi::MCPGetGenericInterface(MocapApi::IMCPSettings_Version, reinterpret_cast<void**>(&mcpSettings)));
  MCPSettingsHandle_t mcpSettingsHandle = 0;
  VERIFY(mcpSettings->CreateSettings(&mcpSettingsHandle));
  VERIFY(mcpSettings->SetSettingsUDP(7012, mcpSettingsHandle));
  VERIFY(mcpSettings->SetSettingsBvhData(MocapApi::BvhDataType_Binary, mcpSettingsHandle));
  VERIFY(mcpSettings->SetSettingsBvhTransformation(MocapApi::BvhTransformation_Enable, mcpSettingsHandle));
  VERIFY(mcpSettings->SetSettingsBvhRotation(MocapApi::BvhRotation_YXZ, mcpSettingsHandle));

  IMCPApplication* mcpApplication = nullptr;
  VERIFY(MCPGetGenericInterface(MocapApi::IMCPApplication_Version, reinterpret_cast<void**>(&mcpApplication)));
  MCPApplicationHandle_t _appcliation = 0;
  VERIFY(mcpApplication->CreateApplication(&_appcliation));
  VERIFY(mcpApplication->SetApplicationSettings(mcpSettingsHandle, _appcliation));
  VERIFY(mcpSettings->DestroySettings(mcpSettingsHandle));
  VERIFY(mcpApplication->OpenApplication(_appcliation));

  while (!DataHandler::getInstance().exit_flag()) {
    std::vector<MCPEvent_t> vEvents;
    uint32_t unEvent = 0;
    VERIFY(mcpApplication->PollApplicationNextEvent(nullptr, &unEvent, _appcliation));
    vEvents.resize(unEvent);
    std::for_each(vEvents.begin(), vEvents.end(), [](MCPEvent_t& ev) { ev.size = sizeof(MCPEvent_t); });
    auto mcpError = mcpApplication->PollApplicationNextEvent(vEvents.data(), &unEvent, _appcliation);
    if (mcpError == Error_None) {
      vEvents.resize(unEvent);
      std::for_each(vEvents.begin(), vEvents.end(), [](const MCPEvent_t& ev) { handleEvent(ev); });
    } else if (mcpError != Error_MoreEvent) {
      std::cout << "\'mcpApplication->PollApplicationNextEvent(nullptr, "
                   "&unEvent, _appcliation)\' return Error : "
                << mcpError << std::endl;
      throw std::runtime_error("");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  VERIFY(mcpApplication->CloseApplication(_appcliation));
  VERIFY(mcpApplication->DestroyApplication(_appcliation));
  return 0;
}

void DataHandler::init() {
  std::vector<std::string> body_names = {"Hips",
                                         "RightUpLeg",
                                         "RightLeg",
                                         "RightFoot",
                                         "LeftUpLeg",
                                         "LeftLeg",
                                         "LeftFoot",
                                         "Spine",
                                         "Spine1",
                                         "Spine2",
                                         "Neck",
                                         "Neck1",
                                         "Head",
                                         "RightShoulder",
                                         "RightArm",
                                         "RightForeArm",
                                         "RightHand",
                                         "RightHandThumb1",
                                         "RightHandThumb2",
                                         "RightHandThumb3",
                                         "RightInHandIndex",
                                         "RightHandIndex1",
                                         "RightHandIndex2",
                                         "RightHandIndex3",
                                         "RightInHandMiddle",
                                         "RightHandMiddle1",
                                         "RightHandMiddle2",
                                         "RightHandMiddle3",
                                         "RightInHandRing",
                                         "RightHandRing1",
                                         "RightHandRing2",
                                         "RightHandRing3",
                                         "RightInHandPinky",
                                         "RightHandPinky1",
                                         "RightHandPinky2",
                                         "RightHandPinky3",
                                         "LeftShoulder",
                                         "LeftArm",
                                         "LeftForeArm",
                                         "LeftHand",
                                         "LeftHandThumb1",
                                         "LeftHandThumb2",
                                         "LeftHandThumb3",
                                         "LeftInHandIndex",
                                         "LeftHandIndex1",
                                         "LeftHandIndex2",
                                         "LeftHandIndex3",
                                         "LeftInHandMiddle",
                                         "LeftHandMiddle1",
                                         "LeftHandMiddle2",
                                         "LeftHandMiddle3",
                                         "LeftInHandRing",
                                         "LeftHandRing1",
                                         "LeftHandRing2",
                                         "LeftHandRing3",
                                         "LeftInHandPinky",
                                         "LeftHandPinky1",
                                         "LeftHandPinky2",
                                         "LeftHandPinky3"};

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = "world";
  transform_stamped.transform.translation.x = 0;
  transform_stamped.transform.translation.y = 0;
  transform_stamped.transform.translation.z = 0;
  transform_stamped.transform.rotation.x = 0;
  transform_stamped.transform.rotation.y = 0;
  transform_stamped.transform.rotation.z = 0;
  transform_stamped.transform.rotation.w = 1;
  int i = 0;
  for (auto& name : body_names) {
    transform_stamped.child_frame_id = name;
    data_.push_back(transform_stamped);

    joint_index_.insert({name, i});
    i++;
  }
  thread_ = new std::thread(run);
}

}  // namespace noitom_mocap
}  // namespace pnd