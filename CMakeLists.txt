cmake_minimum_required(VERSION 3.5)
project(vsa_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

ament_package()
