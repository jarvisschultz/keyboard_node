cmake_minimum_required(VERSION 2.8.3)
project(keyboard_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  puppeteer_msgs
  )

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS puppeteer_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(keyboard_node src/keyboard_main.cpp)
add_executable(multi_keyboard_node src/multi_keyboard.cpp)
add_dependencies(keyboard_node ${catkin_EXPORTED_TARGETS})
add_dependencies(multi_keyboard_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(keyboard_node ${catkin_LIBRARIES})
target_link_libraries(multi_keyboard_node ${catkin_LIBRARIES})
