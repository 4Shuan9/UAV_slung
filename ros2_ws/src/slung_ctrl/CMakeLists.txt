cmake_minimum_required(VERSION 3.8)
project(slung_ctrl)

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
add_executable(slung_takeoff src/slung_takeoff.cpp)
ament_target_dependencies(slung_takeoff rclcpp px4_msgs)

add_executable(slung_antiswing_PD src/slung_antiswing_PD.cpp)
ament_target_dependencies(slung_antiswing_PD rclcpp px4_msgs geometry_msgs std_msgs)
target_link_libraries(slung_antiswing_PD Eigen3::Eigen)

add_executable(slung_antiswing_ADRC src/slung_antiswing_ADRC.cpp)
ament_target_dependencies(slung_antiswing_ADRC rclcpp px4_msgs geometry_msgs std_msgs)
target_link_libraries(slung_antiswing_ADRC Eigen3::Eigen)

add_executable(slung_antiswing_waypoint src/slung_antiswing_waypoint.cpp)
ament_target_dependencies(slung_antiswing_waypoint rclcpp geometry_msgs px4_msgs std_msgs)

add_executable(slung_antiswing_square src/slung_antiswing_square.cpp)
ament_target_dependencies(slung_antiswing_square rclcpp geometry_msgs px4_msgs)

add_executable(slung_antiswing_eight src/slung_antiswing_eight.cpp)
ament_target_dependencies(slung_antiswing_eight rclcpp geometry_msgs px4_msgs)

add_executable(swing_angle_truth src/swing_angle_truth.cpp)
ament_target_dependencies(swing_angle_truth rclcpp geometry_msgs message_filters)
target_link_libraries(swing_angle_truth Eigen3::Eigen)

add_executable(swing_angle_cv src/swing_angle_cv.cpp)
ament_target_dependencies(swing_angle_cv rclcpp geometry_msgs message_filters)
target_link_libraries(swing_angle_cv Eigen3::Eigen)

add_executable(swing_angle_cv_comp src/swing_angle_cv_comp.cpp)
ament_target_dependencies(swing_angle_cv_comp rclcpp geometry_msgs message_filters px4_msgs)
target_link_libraries(swing_angle_cv_comp Eigen3::Eigen)

add_executable(swing_angle_eso src/swing_angle_eso.cpp)
ament_target_dependencies(swing_angle_eso rclcpp geometry_msgs px4_msgs)

add_executable(swing_angle_eso_fusion src/swing_angle_eso_fusion.cpp)
ament_target_dependencies(swing_angle_eso_fusion rclcpp geometry_msgs px4_msgs)

add_executable(swing_angle_ekf src/swing_angle_ekf.cpp)
ament_target_dependencies(swing_angle_ekf rclcpp geometry_msgs px4_msgs)
target_link_libraries(swing_angle_ekf Eigen3::Eigen)

# The ADRC controller and LESO fusion node use the custom interface generated
# in this same package.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(swing_angle_eso_fusion "${cpp_typesupport_target}")
target_link_libraries(slung_antiswing_ADRC "${cpp_typesupport_target}")

add_executable(swing_error_eval src/swing_error_eval.cpp)
ament_target_dependencies(swing_error_eval rclcpp geometry_msgs std_msgs px4_msgs)
target_link_libraries(swing_error_eval "${cpp_typesupport_target}")

# --- 4. Installation ---
install(TARGETS
  slung_takeoff
  slung_antiswing_PD
  slung_antiswing_ADRC
  slung_antiswing_waypoint
  slung_antiswing_square
  slung_antiswing_eight
  swing_angle_truth
  swing_angle_cv
  swing_angle_cv_comp
  swing_angle_eso
  swing_angle_eso_fusion
  swing_angle_ekf
  swing_error_eval
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gz_cam_cv.py
  scripts/vision_time_aligner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# --- 5. Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- 6. Package Export ---
ament_export_dependencies(rosidl_default_runtime)
ament_package()