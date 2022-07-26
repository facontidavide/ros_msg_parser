cmake_minimum_required(VERSION 2.8.3)
project(ros_msg_parser)


# Build flags
#---- Enable C++17 ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

###########
## Build ##
###########

include_directories(
    include
    3rdparty
)

## Declare a C++ library
 add_library(ros_msg_parser
   src/ros_type.cpp
   src/ros_field.cpp
   src/stringtree_leaf.cpp
   src/ros_message.cpp
#   src/ros_parser.cpp
 )


add_executable(test_parser
    test/test_parser.cpp)

target_link_libraries(test_parser ros_msg_parser)


