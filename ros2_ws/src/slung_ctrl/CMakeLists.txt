cmake_minimum_required(VERSION 3.8)
project(slung_ctrl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(include)

# --- 1. Compiler Options ---
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 2. Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- 2.1 Custom interfaces ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SwingADRCState.msg"
  DEPENDENCIES std_msgs
)

# --- 3. C++ Executables ---
add_executable(slung_control_pd src/slung_control_pd.cpp)
ament_target_dependencies(slung_control_pd rclcpp px4_msgs geometry_msgs std_msgs)
target_link_libraries(slung_control_pd Eigen3::Eigen)

add_executable(slung_control_adrc src/slung_control_adrc.cpp)
ament_target_dependencies(slung_control_adrc rclcpp px4_msgs geometry_msgs std_msgs)
target_link_libraries(slung_control_adrc Eigen3::Eigen)

add_executable(trajectory_waypoint src/trajectory_waypoint.cpp)
ament_target_dependencies(trajectory_waypoint rclcpp geometry_msgs px4_msgs std_msgs)

add_executable(trajectory_square src/trajectory_square.cpp)
ament_target_dependencies(trajectory_square rclcpp geometry_msgs px4_msgs)

add_executable(trajectory_eight src/trajectory_eight.cpp)
ament_target_dependencies(trajectory_eight rclcpp geometry_msgs px4_msgs)

add_executable(swing_truth src/swing_truth.cpp)
ament_target_dependencies(swing_truth rclcpp geometry_msgs message_filters)
target_link_libraries(swing_truth Eigen3::Eigen)

add_executable(swing_vision src/swing_vision.cpp)
ament_target_dependencies(swing_vision rclcpp geometry_msgs message_filters)
target_link_libraries(swing_vision Eigen3::Eigen)

add_executable(swing_vision_predictor src/swing_vision_predictor.cpp)
ament_target_dependencies(swing_vision_predictor rclcpp geometry_msgs message_filters px4_msgs)
target_link_libraries(swing_vision_predictor Eigen3::Eigen)

add_executable(swing_eso src/swing_eso.cpp)
ament_target_dependencies(swing_eso rclcpp geometry_msgs px4_msgs)

add_executable(swing_eso_fusion src/swing_eso_fusion.cpp)
ament_target_dependencies(swing_eso_fusion rclcpp geometry_msgs px4_msgs std_msgs)
target_link_libraries(swing_eso_fusion Eigen3::Eigen)

add_executable(swing_ekf2 src/swing_ekf2.cpp)
ament_target_dependencies(swing_ekf2 rclcpp geometry_msgs px4_msgs)
target_link_libraries(swing_ekf2 Eigen3::Eigen)

add_executable(swing_ekf3 src/swing_ekf3.cpp)
ament_target_dependencies(swing_ekf3 rclcpp px4_msgs geometry_msgs std_msgs)
target_link_libraries(swing_ekf3 Eigen3::Eigen)

# The ADRC controller and LESO fusion node use the custom interface generated
# in this same package.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(swing_eso_fusion "${cpp_typesupport_target}")
target_link_libraries(slung_control_adrc "${cpp_typesupport_target}")
target_link_libraries(swing_ekf3 "${cpp_typesupport_target}")

# --- 4. Installation ---
install(TARGETS
  slung_control_pd
  slung_control_adrc
  trajectory_waypoint
  trajectory_square
  trajectory_eight
  swing_truth
  swing_vision
  swing_vision_predictor
  swing_eso
  swing_eso_fusion
  swing_ekf2
  swing_ekf3
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/payload_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# --- 5. Package Export ---
ament_export_dependencies(rosidl_default_runtime)
ament_package()
