cmake_minimum_required(VERSION 3.8)
project(go1_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# No need to find gazebo_ros / ros_gz_* here – we’re not compiling C++.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install only what actually exists in this package
install(
  DIRECTORY
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/contacts_to_wrench.py
  DESTINATION lib/${PROJECT_NAME}
)

# If you later add a models/ directory, uncomment this or keep it OPTIONAL:
# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
#   install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
# endif()

ament_package()

