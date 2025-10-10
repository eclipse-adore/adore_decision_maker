cmake_minimum_required(VERSION 3.8)
project(decision_maker VERSION 0.1.0 LANGUAGES CXX)

# C++20 strict mode
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build flags
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(adore_math REQUIRED)
find_package(adore_map REQUIRED)
find_package(adore_planning REQUIRED)
find_package(adore_ros2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(adore_dynamics_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)  # provides the CMake target 'yaml-cpp'

# Add additional adore_* or ROS 2 dependencies if needed

# Executable node
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/decision_maker.cpp
  src/decision_publisher.cpp

  src/behaviours.cpp
  src/domain.cpp
  src/conditions.cpp
  src/rules.cpp

)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  rclcpp_components
  adore_math
  adore_map
  adore_planning
  adore_ros2_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  adore_dynamics_conversions
  sensor_msgs
  yaml-cpp
  yaml_cpp_vendor
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/rules.yaml
        DESTINATION share/${PROJECT_NAME}/config)

# Component library (without main)
add_library(${PROJECT_NAME}_component SHARED
  src/decision_maker.cpp
)

target_include_directories(${PROJECT_NAME}_component
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_component
  ament_index_cpp
  rclcpp
  rclcpp_components
  adore_math
  adore_map
  adore_planning
  adore_ros2_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  adore_dynamics_conversions
  sensor_msgs
  yaml_cpp_vendor
  yaml-cpp
)
target_link_libraries(${PROJECT_NAME}_component yaml-cpp)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "adore::DecisionMaker"
  EXECUTABLE decisionmaker
)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()



