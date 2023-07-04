cmake_minimum_required(VERSION 3.5)
project(cpp_zeddetect)

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

if($ENV{ROS_DISTRO} STREQUAL "foxy")
  add_compile_options(-DROS_DISTRO=1)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
  add_compile_options(-DROS_DISTRO=2)
else()# eloquent
  add_compile_options(-DROS_DISTRO=0)
endif()

# find dependencies
SET(ENABLE_VS_NODE False CACHE BOOL "Some user-specified option")
if( ENABLE_VS_NODE ) 
  add_compile_options(-DVS_NODE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vehicle_interfaces REQUIRED)
find_package(OpenCV ${VERSION_REQ_OCV} REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

set(src_file
  "src/cameramodule.cpp"
  "src/grounddot.cpp"
  "src/groundline.cpp"
  "src/imageProc.h"
  "src/re_tx1.cpp"
  "src/re_tx1.h")

SET(SPECIAL_OS_LIB "pthread" "X11")
add_definitions(-Wno-write-strings)

add_executable(zed ${src_file})
ament_target_dependencies(zed rclcpp vehicle_interfaces)
target_link_libraries(zed
  ${OpenCV_LIBRARIES}
  X11)

install(TARGETS
  zed
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
