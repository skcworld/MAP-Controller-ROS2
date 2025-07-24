cmake_minimum_required(VERSION 3.8)
project(f110_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필요한 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 메시지 파일 등록
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Wpnt.msg"
  "msg/WpntArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()