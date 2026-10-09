cmake_minimum_required(VERSION 3.10)
project(yasmin_viewer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_msgs REQUIRED)

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})

set(SOURCES
  src/yasmin_viewer/yasmin_viewer_pub.cpp
)

set(DEPENDENCIES
  rclcpp::rclcpp
  yasmin::yasmin
  yasmin_ros::yasmin_ros
  ${yasmin_msgs_TARGETS}
)

set(DEPENDENCIES_PACKAGE
  ament_index_cpp
  rclcpp
  rclcpp_components
  rclpy
  yasmin
  yasmin_ros
  yasmin_msgs
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

# Viewer node as a component, so it can run in a component container
add_library(yasmin_viewer_node_component SHARED
  src/yasmin_viewer/yasmin_viewer_node.cpp
)
target_include_directories(yasmin_viewer_node_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(yasmin_viewer_node_component PUBLIC
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  ${yasmin_msgs_TARGETS}
  ${Boost_LIBRARIES}
)
rclcpp_components_register_nodes(yasmin_viewer_node_component
  "yasmin_viewer::YasminViewerNode")

add_executable(yasmin_viewer_node
  src/yasmin_viewer/main.cpp
)
target_link_libraries(yasmin_viewer_node PUBLIC yasmin_viewer_node_component)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} yasmin_viewer_node
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS yasmin_viewer_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_yasmin_viewer_node test/test_yasmin_viewer_node.cpp
    TIMEOUT 120)
  if(TARGET test_yasmin_viewer_node)
    target_link_libraries(test_yasmin_viewer_node yasmin_viewer_node_component)
  endif()
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES_PACKAGE})

ament_package()
