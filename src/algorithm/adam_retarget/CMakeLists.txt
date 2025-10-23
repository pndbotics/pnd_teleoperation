cmake_minimum_required(VERSION 3.5)
project(adam_retarget)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED )
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# casadi library 
find_package(casadi_vendor REQUIRED )

# set include
include_directories(include)

# executable

add_executable(${CMAKE_PROJECT_NAME} src/main.cpp src/Retarget.cpp)
ament_target_dependencies(${CMAKE_PROJECT_NAME} 
  rclcpp
  tf2_ros
  geometry_msgs
  sensor_msgs
  std_srvs
  casadi_vendor
)



install(TARGETS
  ${CMAKE_PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)


# ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.py"
    PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE # Sets rwxr-xr-x
  # PATTERN ".git" EXCLUDE # Optional: exclude .git or other unwanted files/dirs
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


install(DIRECTORY opti_config/
  DESTINATION share/${PROJECT_NAME}/opti_config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_utils_test test/utils_test.cpp)
  ament_target_dependencies(${PROJECT_NAME}_utils_test casadi_vendor)
  target_include_directories(${PROJECT_NAME}_utils_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_utils_test casadi)
endif()
ament_package()
