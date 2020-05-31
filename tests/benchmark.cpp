#include <boost/algorithm/string.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/utility/string_ref.hpp>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <iostream>
#include <chrono>
#include "ros_msg_parser/ros_parser.hpp"
#include "ros_msg_parser/utils/shape_shifter.hpp"

#include <benchmark/benchmark.h>

using namespace ros::message_traits;
using namespace RosMsgParser;


static void BM_Joints(benchmark::State& state)
{
  RosMsgParser::Parser parser("joint_state",
                              ROSType(DataType<sensor_msgs::JointState>::value()),
                              Definition<sensor_msgs::JointState>::value());

  sensor_msgs::JointState js_msg;

  js_msg.name.resize(6);
  js_msg.position.resize(6);
  js_msg.velocity.resize(6);
  js_msg.effort.resize(6);

  const char* suffix[6] = { "_A", "_B", "_C", "_D" , "_E", "_F"};

  for (size_t i=0; i< js_msg.name.size() ; i++)
  {
    js_msg.header.seq = 100+i;
    js_msg.header.stamp.sec = 1234;
    js_msg.header.frame_id = std::string("frame").append(suffix[i]);

    js_msg.name[i] = std::string("child").append(suffix[i]);
    js_msg.position[i]  = 10 +i;
    js_msg.velocity[i]  = 20 +i;
    js_msg.effort[i]    = 30 +i;
  }

  std::vector<uint8_t> buffer( ros::serialization::serializationLength(js_msg) );
  ros::serialization::OStream stream(buffer.data(), buffer.size());
  ros::serialization::Serializer<sensor_msgs::JointState>::write(stream, js_msg);

  FlatMessage flat_container;
  RenamedValues renamed_vals;

  std::string json;
  while (state.KeepRunning())
  {
//    parser.deserializeIntoFlatMsg(buffer, &flat_container);
//    CreateRenamedValues(flat_container, renamed_vals);

    parser.deserializeIntoJson(buffer, &json);
  }
}

BENCHMARK(BM_Joints);

BENCHMARK_MAIN();

