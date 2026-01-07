cmake_minimum_required(VERSION 3.8)
project(imv_problem_5)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_lifecycle_manager REQUIRED)

macro(add_action_node node_name source_file)
  add_executable(${node_name} src/${source_file})
  ament_target_dependencies(${node_name}
    rclcpp plansys2_executor plansys2_lifecycle_manager
  )
  install(TARGETS ${node_name} DESTINATION lib/${PROJECT_NAME})
endmacro()

# Registrazione degli 8 nodi
add_action_node(move move_action_node.cpp)
add_action_node(pick_up_secure pick_up_secure_action_node.cpp)
add_action_node(pick_up_fragile pick_up_fragile_action_node.cpp)
add_action_node(drop_secure drop_secure_action_node.cpp)
add_action_node(drop_fragile drop_fragile_action_node.cpp)
add_action_node(load_pod load_pod_action_node.cpp)
add_action_node(unload_pod_secure unload_pod_secure_action_node.cpp)
add_action_node(unload_pod_fragile unload_pod_fragile_action_node.cpp)
add_action_node(cool_down cool_down_action_node.cpp)

# Installazione directory launch e pddl
install(DIRECTORY launch pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()