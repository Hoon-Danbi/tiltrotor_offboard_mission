cmake_minimum_required(VERSION 3.8)
project(tiltrotor_offboard_mission)

# 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ 표준 (원하면 17로)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# 실행파일 추가
add_executable(offboard_mission_node
  src/offboard_mission_node.cpp
)

# 타겟에 의존성 연결
ament_target_dependencies(offboard_mission_node
  rclcpp
  px4_msgs
)

# 설치 경로 설정
install(TARGETS
  offboard_mission_node
  DESTINATION lib/${PROJECT_NAME}
)

# 테스트(있으면) 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 라이선스 아직이면 linter 일부 스킵
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament 패키지 매크로 (보통 맨 마지막)
ament_package()
