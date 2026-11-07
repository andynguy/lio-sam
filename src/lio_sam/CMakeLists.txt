cmake_minimum_required(VERSION 3.8)
project(lio_sam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(lio_sam_utils src/MathUtils.cpp)
target_include_directories(lio_sam_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lio_sam_utils PUBLIC Eigen3::Eigen)

add_executable(cloud_processor src/CloudProcessor.cpp src/main.cpp)
target_include_directories(cloud_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cloud_processor
                         rclcpp
                         std_msgs
                         sensor_msgs
                         geometry_msgs)

add_executable(imu_odometry src/ImuOdometry.cpp src/ImuMain.cpp)
target_link_libraries(imu_odometry lio_sam_utils)
ament_target_dependencies(imu_odometry
                         rclcpp
                         sensor_msgs
                         nav_msgs
                         geometry_msgs
                         tf2_eigen)

install(TARGETS
  cloud_processor
  imu_odometry
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

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

  # Unit tests
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_math_utils test/test_math_utils.cpp)
  target_link_libraries(test_math_utils lio_sam_utils)
endif()

ament_package()
