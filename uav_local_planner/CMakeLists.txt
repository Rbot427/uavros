cmake_minimum_required(VERSION 2.8.3)
project(uav_local_planner)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        dynamic_reconfigure
        eigen_conversions
        geometry_msgs
        nav_msgs
        roscpp
        tf
        uav_msgs)

find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(cfg/UAVController.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(uav_local_planner src/local_planner.cpp src/controller.cpp)
target_link_libraries(uav_local_planner ${catkin_LIBRARIES})
add_dependencies(uav_local_planner ${PROJECT_NAME}_gencfg ${uav_local_planner_EXPORTED_TARGETS})

add_executable(twist_local_planner src/twist_local_planner.cpp)
target_link_libraries(twist_local_planner ${catkin_LIBRARIES})
add_dependencies(twist_local_planner ${uav_local_planner_EXPORTED_TARGETS})

install(
    TARGETS uav_local_planner twist_local_planner
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
