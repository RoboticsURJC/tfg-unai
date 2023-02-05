cmake_minimum_required(VERSION 3.5)
project(pibot_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(CMAKE_CXX_STANDARD 17)

#set(dependencies
#    rclcpp
#    rclcpp_action
#    plansys2_msgs
#    nav2_msgs
#    plansys2_domain_expert
#    plansys2_executor
#    plansys2_planner
#    plansys2_problem_expert
#    plansys2_pddl_parser
#    geometry_msgs
#)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include/
)

#add_library(${PROJECT_NAME} SHARED src/MoveAction.cpp)
#ament_target_dependencies(${PROJECT_NAME} ${dependencies})
#
#add_executable(move_action_node src/move_action_node.cpp)
#ament_target_dependencies(move_action_node ${dependencies})
#target_link_libraries(move_action_node ${PROJECT_NAME})


#add_executable(node_test src/node_test.cpp)
#ament_target_dependencies(node_test ${dependencies})
#target_link_libraries(node_test ${PROJECT_NAME})

add_executable(node_test src/node_test.cpp)
target_include_directories(node_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY
  launch
#  pddl
#  config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  node_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#install(TARGETS node_test
#  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})

ament_package()
