cmake_minimum_required(VERSION 3.10)

project(ros_msg_parser  LANGUAGES C CXX VERSION 2.0.0)

# Build flags
#---- Enable C++17 ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)


add_subdirectory(3rdparty/Fast-CDR)

include_directories(
    include
    3rdparty/Fast-CDR/include
)

###############################################

# http://answers.ros.org/question/230877/optionally-build-a-package-with-catkin/
if( CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
    set(COMPILING_WITH_CATKIN 1)
    message(STATUS "Compiling with CATKIN.")

    set(PACKAGE_DEPEND
        roslib
        roscpp
        roscpp_serialization
        rosbag_storage
        sensor_msgs
        geometry_msgs)

    find_package(catkin REQUIRED COMPONENTS
        ${PACKAGE_DEPEND})

    catkin_package(
        INCLUDE_DIRS  include
        LIBRARIES  ros_msg_parser
        CATKIN_DEPENDS ${PACKAGE_DEPEND}
    )

    include_directories(${catkin_INCLUDE_DIRS})

elseif( DEFINED ENV{AMENT_PREFIX_PATH})

    set(COMPILING_WITH_AMENT 1)
    message(STATUS "Compiling with AMENT.")

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rcpputils REQUIRED)
    find_package(rosbag2 REQUIRED)
    find_package(rosbag2_transport REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)

    set( PACKAGE_DEPEND
        ament_cmake
        rclcpp
        rcpputils
        rosbag2
        rosbag2_transport
        sensor_msgs
        geometry_msgs )

else()
    message(STATUS "Compiling without ROS")
endif()

###############################################


## Declare a C++ library
 add_library(ros_msg_parser
   src/ros_type.cpp
   src/ros_field.cpp
   src/stringtree_leaf.cpp
   src/ros_message.cpp
   src/ros_parser.cpp
   src/deserializer.cpp
 )

if(COMPILING_WITH_CATKIN)

    add_executable(test_parser
        test/test_parser.cpp
        test/test_ros1.cpp )

    target_link_libraries(test_parser
        ros_msg_parser
        ${catkin_LIBRARIES})

    install(TARGETS test_parser
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

elseif(COMPILING_WITH_AMENT)

else()
    add_executable(test_parser
        test/test_parser.cpp)
    target_link_libraries(test_parser
        ros_msg_parser)
endif()

