
cmake_minimum_required(VERSION 3.8)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(lidar_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
#ros 2
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(rviz2 REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# gazebo 
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
# set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-common4 REQUIRED COMPONENTS profiler)
# set(GZ_COMMON_VER ${ignition-common4_VERSION_MAJOR})
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})


add_library(random_mover SHARED src/random_mover.cc)

add_library(joint_random_mover SHARED src/joint_random_mover.cc)

# add_library(random_scaler SHARED src/random_scaler.cc)

target_include_directories(
  random_mover PRIVATE include
)

target_include_directories(
  joint_random_mover PRIVATE include
)

# target_include_directories(
#   random_scaler PRIVATE include
# )

set_property(TARGET random_mover joint_random_mover PROPERTY CXX_STANDARD 17)

target_link_libraries(random_mover 
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER}
)

target_link_libraries(joint_random_mover 
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER}
)

# target_link_libraries(random_scaler
#   ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
#   ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
#   ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER}
# )

install(
  TARGETS random_mover joint_random_mover #random_scaler
  DESTINATION $ENV{HOME}/.ignition/gazebo/plugins
)


install(DIRECTORY 
  models
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  lidar_sim
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  configs
  DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/ign_hooks.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/ign_hooks.sh.in")


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
# LIST(APPEND IGN_GAZEBO_SYSTEM_PLUGIN_PATH lib/${PROJECT_NAME})
ament_package()
