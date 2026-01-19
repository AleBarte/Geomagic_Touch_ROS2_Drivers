cmake_minimum_required(VERSION 3.8)
project(omni_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(omni_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_path(OPENHAPTICS_INCLUDE_DIR NAMES HD/hd.h HL/hl.h HDU/hdu.h)


find_library(OPENHAPTICS_HD_LIB  NAMES HD)
find_library(OPENHAPTICS_HDU_LIB NAMES HDU)
find_library(OPENHAPTICS_HL_LIB  NAMES HL)

if(NOT OPENHAPTICS_INCLUDE_DIR OR NOT OPENHAPTICS_HD_LIB OR NOT OPENHAPTICS_HDU_LIB)
  message(FATAL_ERROR "OpenHaptics (HD/HDU/HL) not found. Set include/lib paths or install OpenHaptics.")
endif()


add_executable(master
  src/omni_state.cpp
  src/omni_state_main.cpp
)

target_include_directories(master PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OPENHAPTICS_INCLUDE_DIR}
)

target_link_libraries(
  master
  ${OPENHAPTICS_HD_LIB}
  ${OPENHAPTICS_HDU_LIB}
  ncurses
  rt
  pthread
)

ament_target_dependencies(master
  rclcpp
  geometry_msgs
  sensor_msgs
  Eigen3
  std_msgs
  omni_msgs
)

install(TARGETS master
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
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
