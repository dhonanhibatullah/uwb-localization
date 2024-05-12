cmake_minimum_required(VERSION 3.8)
project(uwbloc_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position3D.msg"
  "msg/AnchorInfo.msg"
  "msg/ServerCall.msg"
  "msg/TagBroadcastRX.msg"
  "msg/TagBroadcastTX.msg"
  "msg/CalibrationPollRX.msg"
  "msg/CalibrationPollTX.msg"
  "msg/CalibrationRespRX.msg"
  "msg/CalibrationRespTX.msg"
  "msg/MasterClockSyncRX.msg"
  "msg/MasterClockSyncTX.msg"
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} 
  rosidl_typesupport_cpp
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
