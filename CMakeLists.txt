cmake_minimum_required(VERSION 3.22)
project(eskf_lio)

cmake_policy(SET CMP0072 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Open3D REQUIRED HINTS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/open3d/lib/cmake/Open3D)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OpenMP_CXX_INCLUDE_DIRS}
)

add_library(eskf_lio
  src/Odometry.cpp
  src/ErrorStateKF.cpp
  src/LocalMap.cpp
)

target_link_libraries(eskf_lio 
  Eigen3::Eigen
  Open3D::Open3D
  ${YAML_CPP_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_executable(eskf_lio_node
  src/main.cpp
)

ament_target_dependencies(eskf_lio_node
  rclcpp
  sensor_msgs
)

target_link_libraries(eskf_lio_node
  eskf_lio
)

target_compile_definitions(eskf_lio_node PUBLIC
  CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/hilti_config.yaml"
)

install(TARGETS
  eskf_lio_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_localmap_test test/localmap_test.cpp)
  target_link_libraries(${PROJECT_NAME}_localmap_test
    eskf_lio
  )
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
endif()

ament_package()
