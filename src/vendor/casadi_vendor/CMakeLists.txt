cmake_minimum_required(VERSION 3.8)
project(casadi_vendor)

option(USE_ISOLATE_CASADI "Use ExternalProject to build CasADi from source instead of find in system" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(casadi REQUIRED)

if(NOT USE_ISOLATE_CASADI)
  find_package(casadi REQUIRED)
else()
  include(ExternalProject)

  ExternalProject_Add(casadi_external
    GIT_REPOSITORY https://github.com/casadi/casadi.git
    GIT_TAG 3.7.0  # 或特定版本
    GIT_SHALLOW TRUE
    CMAKE_ARGS
      -DWITH_PYTHON=OFF
      -DWITH_BUILD_REQUIRED=ON
      -DWITH_IPOPT=ON 
      -DWITH_BUILD_IPOPT=ON
      -DWITH_MUMPS=ON
      -DWITH_BUILD_MUMPS=ON
      -DWITH_OPENMP=ON 
      -DWITH_THREAD=ON
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/casadi_vendor
    UPDATE_COMMAND ""
  )
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/casadi_vendor/
    DESTINATION ${CMAKE_INSTALL_PREFIX}
  )
endif()

# ament_export_include_directories(include) 
ament_export_libraries(casadi ipopt)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
