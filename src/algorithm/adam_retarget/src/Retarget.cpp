#include "adam_retarget/Retarget.hpp"

using json = nlohmann::json;
const std::unordered_map<std::string, FunctionType> function_type_map = {{"Serialization", FunctionType::SERIAL},
                                                                         {"CodeGeneration", FunctionType::CODEGEN}};

RetargetOptimization::RetargetOptimization(std::string casadi_function_path, const rclcpp::Logger& logger)
    : m_logger(m_logger) {
  // load example io data from json
  json retarget_config;
  std::filesystem::path config_path(casadi_function_path);
  m_working_directory = config_path.parent_path();

  std::ifstream retarget_config_file(casadi_function_path);

  if (!retarget_config_file.is_open()) {
    RCLCPP_ERROR(m_logger, "Error: Could not open file %s", casadi_function_path.c_str());
  } else {
    RCLCPP_INFO(m_logger, "File opened successfully");
  }

  retarget_config_file >> retarget_config;
  retarget_config_file.close();

  auto opti_func_str_list = retarget_config["opti_function"];
  m_labels = retarget_config["extend_labels"];
  // try to find export_method property(if not found, use default Serialization) enum[CodeGeneration Serialization]
  std::string export_method = "Serialization";

  if (retarget_config.find("export_method") != retarget_config.end()) {
    export_method = retarget_config["export_method"].get<std::string>();
    RCLCPP_INFO(m_logger, "export_method: %s", export_method.c_str());
  } else {
    RCLCPP_INFO(m_logger, "export_method not found, using default Serialization");
  }
  if (function_type_map.find(export_method) == function_type_map.end()) {
    RCLCPP_ERROR(m_logger, "Invalid export method: %s", export_method.c_str());
    throw std::runtime_error("Invalid export method: " + export_method);
  }
  m_function_type = function_type_map.at(export_method);

  // build idx map
  for (size_t i = 0; i < m_labels.size(); i++) {
    m_label_idx_map[m_labels[i]] = i;
  }

  for (json::iterator it = opti_func_str_list.begin(); it != opti_func_str_list.end(); ++it) {
    m_opti_functions_list.push_back(_load_function(it.value()));
  }

  auto update_result_func_str_list = retarget_config["update_result_function"];
  for (json::iterator it = update_result_func_str_list.begin(); it != update_result_func_str_list.end(); ++it) {
    m_update_result_functions_list.push_back(_load_function(it.value()));
  }

  auto post_process_function_str = retarget_config["extend_function"].get<std::string>();
  m_post_process_function = _load_function(post_process_function_str);

  auto params_cal_function_str = retarget_config["params_calculate_function"].get<std::string>();
  m_cal_params_function = _load_function(params_cal_function_str);

  m_result_list.resize(m_opti_functions_list.size());
  m_last_q_list.resize(m_opti_functions_list.size());
  m_last_v_list.resize(m_opti_functions_list.size());
  // m_last_a_list.resize(m_opti_functions_list.size());
  // load zero pose
  auto q0_list = retarget_config["q0"].get<std::vector<std::vector<double>>>();
  for (size_t i = 0; i < q0_list.size(); i++) {
    m_zero_pose.push_back(casadi::DM(q0_list[i]));
    m_last_q_list[i] = casadi::DM(q0_list[i]);
  }
  // auto v0_list = retarget_config["v0"].get<std::vector<std::vector<double>>>();
  // for (size_t i = 0; i < v0_list.size(); i++) {
  //   m_last_v_list[i] = casadi::DM(v0_list[i]);
  //   m_last_a_list[i] = casadi::DM::zeros(v0_list[i].size());
  // }
  // // load opti v limit
  // auto v_limit_list = retarget_config["v_limit"].get<std::vector<std::vector<double>>>();
  // for (size_t i = 0; i < v_limit_list.size(); i++) {
  //   auto v_limit_i = v_limit_list[i];
  //   // replace all negitive element in v_limit_i with inf
  //   for (auto& v : v_limit_i) {
  //     if (v < 0) {
  //       v = std::numeric_limits<double>::infinity();
  //     }
  //   }

  //   casadi::DM vl_dm = casadi::DM(v_limit_i);

  //   m_v_limit_list.push_back(vl_dm);
  //   // RCLCPP_INFO(m_logger, "v_limit[%zu]: %s", i, vl_dm.get_str().c_str());
  // }
  auto default_weights = retarget_config["default_weights"].get<std::unordered_map<std::string, double>>();
  for (auto it = default_weights.begin(); it != default_weights.end(); it++) {
    m_default_weights[it->first] = casadi::DM(it->second);
  }

  auto full_q0_vec = retarget_config["full_q0"].get<std::vector<double>>();
  m_full_q0 = casadi::DM(full_q0_vec);

  auto full_v0_vec = retarget_config["full_v0"].get<std::vector<double>>();
  m_full_v0 = casadi::DM(full_v0_vec);

  // test extend function
  casadi::DMDict init_joint_config;
  init_joint_config["q"] = m_full_q0;
  init_joint_config["v"] = m_full_v0;
  casadi::DMDict init_pose_result;
  m_post_process_function.call(init_joint_config, init_pose_result, true);
  m_full_result = init_pose_result.at("result");

  m_thread_pool = std::make_shared<ThreadPool>(m_opti_functions_list.size());
}

RetargetOptimization::~RetargetOptimization() {}

casadi::DMDict RetargetOptimization::solve_function(casadi::Function& opti_function, casadi::DM& q0,
                                                    casadi::DMDict& params) {
  casadi::DMDict casadi_params;
  casadi::DMDict result;
  for (size_t in_idx = 0; in_idx < opti_function.n_in(); in_idx++) {
    auto input_name = opti_function.name_in(in_idx);
    auto input_shape1 = opti_function.size1_in(in_idx);
    auto input_shape2 = opti_function.size2_in(in_idx);
    if (input_name == "q0") {
      casadi_params["q0"] = q0;
      // std::cout << input_name << ": " << casadi_params[input_name] << std::endl;
      // std::cout << "func size "<< input_shape1<< ", "<< input_shape2 <<" params "<<
      // casadi_params[input_name].size()<<std::endl;
      continue;
    }

    if (params.find(input_name) == params.end()) {
      if (m_default_weights.find(input_name) == m_default_weights.end())
        throw std::runtime_error("Key not found in params: " + input_name);

      params[input_name] = m_default_weights[input_name];
    }

    casadi_params[input_name] = params[input_name];

    // std::cout << input_name << ": " << casadi_params[input_name] << std::endl;
    // std::cout << "func size "<< input_shape1<< ", "<< input_shape2 <<" params "<<
    // casadi_params[input_name].size()<<std::endl;
  }
  // std::cout << "q0: " << q0_list[i] << std::endl;
  opti_function.call(casadi_params, result, true);

  return result;
}

void RetargetOptimization::solve(RetargetParams params, std::vector<casadi::DM> q0_list, float dt, float vel_percent) {
  casadi::DMDict extended_result;
  // walk through the map and add the parameters to the casadi_params
  params["dt"] = casadi::DM(dt);
  params["dq_vel_limit_factor"] = casadi::DM(vel_percent);

  std::vector<std::future<casadi::DMDict>> results_future_list;
  if (m_opti_functions_list.size() > 1) {
    for (size_t i = 0; i < m_opti_functions_list.size(); i++) {
      results_future_list.emplace_back(m_thread_pool->enqueue(
          [this, i, &q0_list, &params] { return solve_function(m_opti_functions_list[i], q0_list[i], params); }));
    }

    {
      casadi::DMDict full_result;
      casadi::DM full_q, full_v;
      full_q = m_full_q0;
      full_v = m_full_v0;
      // update result
      for (size_t i = 0; i < m_opti_functions_list.size(); i++) {
        m_result_list[i] = results_future_list[i].get();
        casadi::DMDict params_i, updated_result_i;
        params_i["full_q"] = full_q;
        params_i["full_v"] = full_v;
        params_i["result_i_q"] = m_result_list[i].at("q");
        params_i["result_i_v"] = m_result_list[i].at("v");

        m_last_q_list[i] = m_result_list[i].at("q");
        m_last_v_list[i] = m_result_list[i].at("v");

        m_update_result_functions_list[i].call(params_i, updated_result_i);
        full_q = updated_result_i.at("updated_q");
        full_v = updated_result_i.at("updated_v");
      }

      full_result["q"] = full_q;
      full_result["v"] = full_v;
      m_post_process_function.call(full_result, extended_result, true);
    }
  } else {
    m_result_list[0] = solve_function(m_opti_functions_list[0], q0_list[0], params);
    m_last_q_list[0] = m_result_list[0].at("q");
    m_last_v_list[0] = m_result_list[0].at("v");

    m_post_process_function.call(m_result_list[0], extended_result, true);
  }

  m_full_result = extended_result.at("result");
}

std::vector<std::string> RetargetOptimization::getLabels() { return m_labels; }

double RetargetOptimization::getResult(std::string key) { return double(m_full_result(m_label_idx_map[key])); }

std::vector<double> RetargetOptimization::getResult(std::vector<std::string> keys) {
  std::vector<double> result_vec;
  for (size_t i = 0; i < keys.size(); i++) {
    result_vec.push_back(double(m_full_result(m_label_idx_map[keys[i]])));
  }
  return result_vec;
}

casadi::DM RetargetOptimization::getResult() { return m_full_result; }

std::vector<casadi::DM> RetargetOptimization::getZeroPose() { return m_zero_pose; }

std::vector<casadi::DM> RetargetOptimization::getLastq0() { return m_last_q_list; }

casadi::DMDict RetargetOptimization::calculate_params(casadi::DMDict& bone_map) {
  casadi::DMDict params;
  for (size_t in_idx = 0; in_idx < m_cal_params_function.n_in(); in_idx++) {
    auto input_name = m_cal_params_function.name_in(in_idx);
    if (bone_map.find(input_name) == bone_map.end()) {
      throw std::runtime_error("Key not found in bone_map: " + input_name);
    }
    params[input_name] = bone_map[input_name];
  }
  casadi::DMDict result;
  m_cal_params_function.call(params, result, true);
  return result;
}

std::vector<std::string> RetargetOptimization::boneNames() {
  std::vector<std::string> m_labels;
  for (size_t in_idx = 0; in_idx < m_cal_params_function.n_in(); in_idx++) {
    auto input_name = m_cal_params_function.name_in(in_idx);
    m_labels.push_back(input_name);
  }
  return m_labels;
}

void RetargetOptimization::setDefaultWeights(casadi::DMDict& weights) {
  for (auto it = weights.begin(); it != weights.end(); it++) {
    m_default_weights[it->first] = it->second;
  }
}

casadi::Function RetargetOptimization::_load_function(std::string function_str) {
  switch (m_function_type) {
    case FunctionType::SERIAL: {
      return casadi::Function::deserialize(function_str);
      break;
    }

    case FunctionType::CODEGEN: {
      auto external_so_path = m_working_directory / function_str;
      // get basename of the file as function name
      auto basename = external_so_path.stem().string();
      return casadi::external(basename, external_so_path.string());
      break;
    }

    default:
      throw std::runtime_error("Invalid function type");
      break;
  }
}
