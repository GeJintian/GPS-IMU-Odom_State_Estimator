cmake_minimum_required(VERSION 3.10)
project(StateEstimator)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package (Boost REQUIRED COMPONENTS system filesystem thread chrono date_time atomic serialization regex timer)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(a2rl_bs_msgs REQUIRED)
find_package(vectornav_msgs REQUIRED)
#find_package(cmake_modules REQUIRED)
# include_directories(include ${catkin_INCLUDE_DIRS})
# include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
find_package(GTSAM REQUIRED)
find_package(TBB)

add_library(${PROJECT_NAME} SHARED)
# if(GTSAM_FOUND)
#   find_package(TBB)
#   include_directories(SYSTEM ${GTSAM_INCLUDE_DIR} "/usr/local/include" ${Boost_INCLUDE_DIRS}) 

#   add_executable(StateEstimator StateEstimator.cpp)
# message(==========${GeographicLib_LIBRARY_DIRS}/${GeographicLib_LIBRARIES})
#   target_link_libraries(StateEstimator ${catkin_LIBRARIES} ${ROS_LIBRARIES} gtsam ${GeographicLib_LIBRARY_DIRS}/libGeographic.so ${TBB_LIBRARIES} ar_diagnostics ${Boost_LIBRARIES})
#   add_dependencies(StateEstimator autorally_msgs_gencpp)

#   install(TARGETS StateEstimator
#           ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#           LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#           RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# endif()

#=====================================================

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    "rclcpp"
    "std_msgs"
    "a2rl_bs_msgs"
    # "tf2" 
    # "tf2_ros" 
    # "tf2_geometry_msgs" 
    # "pcl_conversions" 
    # "PCL" 
    "sensor_msgs" 
    "geometry_msgs"
    "vectornav_msgs"
)

target_sources(${PROJECT_NAME}
  PRIVATE
    "src/StateEstimator.cpp"
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)


set(INCLUDE_DIRS include 
${ament_cmake_INCLUDE_DIRS} 
${rclcpp_INCLUDE_DIRS}
# ${pcl_ros_INCLUDE_DIRS} 
# ${pcl_conversions_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${std_msgs_INCLUDE_DIRS}
${sensor_msgs_INCLUDE_DIRS}
${rclcpp_components_INCLUDE_DIRS}
${GTSAM_INCLUDE_DIRS}
${GeographicLib_INCLUDE_DIRS}
${Boost_LIBRARY_DIRS}
)
# Include
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS 
${ament_cmake_LIBRARY_DIRS} 
${rclcpp_LIBRARY_DIRS}
# ${pcl_ros_LIBRARY_DIRS} 
# ${pcl_conversions_LIBRARY_DIRS}
${std_msgs_LIBRARY_DIRS}
${sensor_msgs_LIBRARY_DIRS}
${rclcpp_components_LIBRARY_DIRS})
#  Linking
link_directories(${LIBRARY_DIRS})

set(LIBS 
${ament_cmake_LIBRARIES} 
${rclcpp_LIBRARIES} 
# ${pcl_ros_LIBRARIES} 
# ${pcl_conversions_LIBRARIES} 
${std_msgs_LIBRARIES} 
${sensor_msgs_LIBRARIES}
${rclcpp_components_LIBRARIES})


add_executable(StateEstimator_exec src/StateEstimatorNode.cpp)
target_link_libraries(StateEstimator_exec ${LIBS} 
# ${PCL_LIBRARIES} 
${PROJECT_NAME} ${GeographicLib_LIBRARIES} gtsam ${TBB_LIBRARIES} ${Boost_LIBRARY_DIRS})

install(TARGETS
${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

# ==== Install ====
install(TARGETS StateEstimator_exec 
ARCHIVE
LIBRARY
RUNTIME
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
# ament_export_dependencies(pcl_ros)
# ament_export_dependencies(pcl_conversions)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(${INCLUDE_DIRS})
ament_package()