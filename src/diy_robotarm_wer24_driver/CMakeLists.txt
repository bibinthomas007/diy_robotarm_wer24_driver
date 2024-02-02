cmake_minimum_required(VERSION 3.8)
project(diy_robotarm_wer24_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(diy_robotarm_wer24_description REQUIRED)
find_package(diy_soft_gripper_description REQUIRED)
find_package(diy_robot_full_cell_description REQUIRED)
find_package(xacro REQUIRED)

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

#add self written folders to package build

install(DIRECTORY
  launch
  config
  incl
  src
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
