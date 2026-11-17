cmake_minimum_required(VERSION 3.14)
project(amcl_3d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(amcl_3d_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)

set(DEPENDENCIES
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_ros
)

add_library(${PROJECT_NAME}_component SHARED
  src/amcl_3d_node.cpp
  src/conversions.cpp
  src/parameters.cpp
)
target_compile_options(${PROJECT_NAME}_component PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_component ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_component amcl_3d_core::amcl_3d_core ${PCL_LIBRARIES})
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "amcl_3d::Amcl3dNode"
  EXECUTABLE amcl_3d_node
)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME} test/test_amcl_3d_node.cpp TIMEOUT 120)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME}_component)
endif()

ament_package()
