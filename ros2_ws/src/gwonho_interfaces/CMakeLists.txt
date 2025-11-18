cmake_minimum_required(VERSION 3.8)
project(gwonho_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 인터페이스에서 참조할 메시지 패키지들을 먼저 find
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 액션/메시지/서비스 파일 등록
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RunMotion.action"
  "srv/WaitForWeight.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
