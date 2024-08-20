cmake_minimum_required(VERSION 3.8)
project(voronoi_planner)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(OpenCV 4.7.0 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(toppra REQUIRED)
find_package(visualization_msgs REQUIRED)

# Configure Voronoi Planner node source files
file(GLOB VORONOI_PLANNER_NODE_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/voronoi_planner/*.cpp")

# Voronoi Planner node
add_library(voronoi_planner SHARED ${VORONOI_PLANNER_NODE_SRCS})
target_compile_definitions(voronoi_planner PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(voronoi_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(voronoi_planner
  Eigen3::Eigen
  ${OpenCV_LIBS}
  qhullcpp
  qhull_r
  Python3::Python
  Python3::Module
  toppra::toppra)
ament_target_dependencies(voronoi_planner
  rclcpp
  rclcpp_components
  visualization_msgs)
rclcpp_components_register_nodes(voronoi_planner "VoronoiPlanner::VoronoiPlannerNode")

# Voronoi Planner standalone application
add_executable(voronoi_planner_app
  src/voronoi_planner_app.cpp)
target_include_directories(voronoi_planner_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(voronoi_planner_app voronoi_planner)
ament_target_dependencies(voronoi_planner_app
  rclcpp)

# Install Voronoi Planner component
install(TARGETS voronoi_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install Voronoi Planner app
install(TARGETS voronoi_planner_app
  DESTINATION lib/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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

# Make sure that the library path is exported even if the library itself is not
ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")

ament_package()
