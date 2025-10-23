#pragma once
#include <casadi/casadi.hpp>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "thread_pool/ThreadPool.h"
typedef std::pair<casadi::DM, casadi::DM> RetargetResult;
typedef std::map<std::string, casadi::DM> RetargetParams;
enum FunctionType { SERIAL, CODEGEN };

class RetargetOptimization {
 public:
  RetargetOptimization(std::string config_json_path, const rclcpp::Logger &logger);
  ~RetargetOptimization();
  void solve(RetargetParams params, std::vector<casadi::DM> q0_list, float dt, float vel_percent);
  std::vector<std::string> getLabels();
  std::vector<casadi::DM> getZeroPose();
  std::vector<casadi::DM> getLastq0();
  casadi::DM getResult();
  double getResult(std::string key);
  std::vector<double> getResult(std::vector<std::string> keys);

  casadi::DMDict calculate_params(casadi::DMDict &bone_map);
  std::vector<std::string> boneNames();

  void setDefaultWeights(casadi::DMDict &weights);
  void setDefaultWeight(std::string name, double value) { m_default_weights[name] = casadi::DM(value); }
  casadi::DMDict getDefaultWeights() { return m_default_weights; }

 private:
  casadi::DMDict solve_function(casadi::Function &opti_function, casadi::DM &q0, casadi::DMDict &params);

  std::vector<casadi::Function> m_opti_functions_list;
  std::vector<casadi::Function> m_update_result_functions_list;

  std::vector<casadi::DMDict> m_result_list;

  casadi::Function m_post_process_function;
  casadi::Function m_cal_params_function;
  std::vector<std::string> m_labels;
  std::map<std::string, size_t> m_label_idx_map;
  std::vector<casadi::DM> m_zero_pose;
  casadi::DMDict m_default_weights;

  casadi::DM m_full_q0;
  casadi::DM m_full_v0;

  std::vector<casadi::DM> m_last_q_list, m_last_v_list, m_last_a_list;
  std::vector<casadi::DM> m_v_limit_list;
  std::shared_ptr<ThreadPool> m_thread_pool;

  casadi::DM m_full_result;

  FunctionType m_function_type;
  // default working directory
  std::filesystem::path m_working_directory;
  rclcpp::Logger m_logger;

  casadi::Function _load_function(std::string function_str);
};
