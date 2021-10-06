cmake_minimum_required(VERSION 2.8.3)
project(ros_msg_parser)

find_package(Boost REQUIRED COMPONENTS regex)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    sensor_msgs
    roscpp
    roscpp_serialization
    rosbag_storage
)

# Build flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 ")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_msg_parser
  CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    roscpp
    roscpp_serialization
    rosbag_storage
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
    include
    3rdparty
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(ros_msg_parser

   src/ros_type.cpp
   src/ros_field.cpp
   src/stringtree_leaf.cpp
   src/ros_message.cpp
   src/ros_parser.cpp
 )

target_link_libraries(ros_msg_parser ${catkin_LIBRARIES})


find_package(benchmark CONFIG)
if (benchmark_FOUND)

    add_executable(ros_parser_benchmark tests/benchmark.cpp)
    add_dependencies(ros_parser_benchmark
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})

    target_link_libraries(ros_parser_benchmark
        ros_msg_parser
        ${catkin_LIBRARIES}
        benchmark::benchmark
        pthread
        )
endif(benchmark_FOUND)

add_executable(simple_example          example/simple.cpp)
target_link_libraries(simple_example   ros_msg_parser ${catkin_LIBRARIES})

add_executable(rosbag_example          example/rosbag_example.cpp)
target_link_libraries(rosbag_example   ros_msg_parser ${catkin_LIBRARIES})

add_executable(generic_subscriber        example/generic_subscriber.cpp)
target_link_libraries(generic_subscriber ros_msg_parser ${catkin_LIBRARIES})

add_executable(selective_deserialization example/selective_deserialization.cpp)
target_link_libraries(selective_deserialization ros_msg_parser ${catkin_LIBRARIES})


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  add_executable(ros_msg_parsing_test
    tests/deserializer_test.cpp
  )

  target_link_libraries(ros_msg_parsing_test
    ros_msg_parser
    gtest
    ${catkin_LIBRARIES}
  )
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ros_msg_parser
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".git" EXCLUDE
)

