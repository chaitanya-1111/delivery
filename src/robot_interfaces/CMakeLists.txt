cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# --- ADDED BLOCK START ---
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FaceBox.msg"
  "msg/FaceTarget.msg"
  "msg/SessionState.msg"
  "msg/SpeechText.msg"
  "msg/Intent.msg"
  "msg/AIRequest.msg"
  "msg/AIResponse.msg"

)
# --- ADDED BLOCK END ---

ament_package()