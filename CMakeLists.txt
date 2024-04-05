cmake_minimum_required(VERSION 3.8)
project(simple)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

add_compile_options(-O3)

include_directories("src/")
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nanoflann REQUIRED)
find_package(TBB REQUIRED)
find_package(dlib REQUIRED)
find_package(Threads REQUIRED)

add_executable(simple src/main.cpp src/ObjectiveFunction.cpp src/Scan.cpp src/Map.cpp src/Register.cpp src/utils.cpp)
ament_target_dependencies(simple rclcpp)

target_link_libraries(simple nanoflann::nanoflann)
target_link_libraries(simple dlib::dlib)
target_link_libraries(simple TBB::tbb)

install(TARGETS
  simple
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
