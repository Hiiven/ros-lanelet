cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

add_compile_options(-std=c++14)
add_compile_options(-fconcepts)
# add_compile_options(-std=c++11) 
#add_compile_options(-std=gnu++11)
add_compile_options(-fext-numeric-literals)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")
set(CMAKE_CXX_STANDARD 14) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  geometry_msgs
  lanelet2_extension
  roscpp
  tf2_ros
  tf2_geometry_msgs
  lanelet2_routing
)

add_service_files(
  FILES
  GetGlobalCoverage.srv # 添加这一行，列出你的服务文件
  # 如果有其他服务文件，也在这里添加
)

# 在 add_service_files 之后，声明依赖的消息包
generate_messages(
  DEPENDENCIES
  std_msgs  # 添加这一行，因为服务定义中使用了 std_msgs 里的类型
  # 如果服务定义中使用了其他包的消息类型，也在这里添加
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS autoware_planning_msgs lanelet2_extension roscpp message_runtime tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mission_planner
  lib/mission_planner_base.cpp
  src/mission_planner_lanelet2/mission_planner_lanelet2.cpp
  src/mission_planner_lanelet2/route_handler.cpp
  src/mission_planner_lanelet2/mission_planner_main.cpp
  src/mission_planner_lanelet2/utility_functions.cpp
)

add_executable(multi_vehicle_coordinator_node # 新的目标名称，也是生成的可执行文件名
  src/mission_planner_lanelet2/multi_vehicle_coordinator_node.cpp # 只包含协调器的源文件
)

add_dependencies(mission_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_dependencies(multi_vehicle_coordinator_node # 为新目标也添加依赖
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(mission_planner 
  ${catkin_LIBRARIES}
)

target_link_libraries(multi_vehicle_coordinator_node # 为 multi_vehicle_coordinator_node 目标链接库
  ${catkin_LIBRARIES}
)

install(TARGETS mission_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
