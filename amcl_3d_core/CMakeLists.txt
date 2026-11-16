cmake_minimum_required(VERSION 3.14)
# C is required by the MPI dependency that VTK pulls in through PCL registration
project(amcl_3d_core VERSION 0.1.0 LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree registration)
find_package(OpenMP)

# The library itself is plain C++ (Eigen + PCL); ROS is not required.
add_library(${PROJECT_NAME} SHARED
  src/gaussian_pose_sampler.cpp
  src/ground_pose_sampler.cpp
  src/localizer.cpp
  src/map/distance_field_map.cpp
  src/map/kdtree_map.cpp
  src/math.cpp
  src/measurement_model/point_cloud_measurement_model.cpp
  src/measurement_model/pose_measurement_model.cpp
  src/motion_model/twist_motion_model.cpp
  src/particle_filter.cpp
  src/particle_set.cpp
  src/random.cpp
  src/resampler/kld_resampler.cpp
  src/resampler/systematic_resampler.cpp
  src/scan_filter.cpp
  src/scan_matcher/ndt_scan_matcher.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
else()
  target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unknown-pragmas)
endif()

set(TEST_SOURCES
  test/test_map.cpp
  test/test_math.cpp
  test/test_particle_filter.cpp
  test/test_motion_model.cpp
  test/test_resampler.cpp
  test/test_scan_filter.cpp
  test/test_localizer.cpp
)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3 PCL)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_${PROJECT_NAME} ${TEST_SOURCES} TIMEOUT 120)
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  endif()
  ament_package()
else()
  # Standalone build (no ROS): only the library and its tests.
  option(BUILD_TESTING "Build tests" ON)
  if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    enable_testing()
    add_executable(test_${PROJECT_NAME} ${TEST_SOURCES})
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} GTest::gtest_main)
    add_test(NAME test_${PROJECT_NAME} COMMAND test_${PROJECT_NAME})
  endif()
endif()
