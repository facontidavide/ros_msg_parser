// You are not supposed to be able to include this header file
// Otherwise you would not need ros_type_introspection at all
// This is just an example
#include <geometry_msgs/Pose.h>
#include <ros_msg_parser/ros_parser.hpp>

// This function works only if type Message is known at
// compilation time. This is not true in most of the use cases
// we care about.
template<typename Message>
void ParseAndDeserialize(const Message& sample, const std::string& topic_name)
{
  using namespace RosMsgParser;
  using namespace ros::message_traits;

  RosMsgParser::Parser parser(topic_name,
        ROSType(DataType<Message>::value()),
        Definition<Message>::value());

  //Serialize the "raw" message into a buffer
  size_t msg_length = ros::serialization::serializationLength(sample);
  std::vector<uint8_t> buffer(msg_length);
  ros::serialization::OStream stream(buffer.data(), buffer.size());
  ros::serialization::serialize(stream, sample);

  // A data structure that contains the deserialized information.
  // It is not meant to be human readable, but it is easy to process...
  FlatMessage flat_container;
  parser.deserializeIntoFlatMsg( Span<uint8_t>(buffer),
                                       &flat_container );

  // Print the data.
  // The type StringTreeLeaf can be converted to human readable
  // string with the method toStdString(),
  // Instead Variant can be casted using the method convert<>()
  for(auto& it: flat_container.value) {
      std::cout << it.first.toStdString() << " >> "
                << it.second.convert<double>() << std::endl;
  }
  for(auto& it: flat_container.name) {
      std::cout << it.first.toStdString() << " >> "
                << it.second << std::endl;
  }
}

int main(int argc, char **argv)
{
  geometry_msgs::Pose pose_sample;

  pose_sample.position.x = 1;
  pose_sample.position.y = 2;
  pose_sample.position.z = 3;

  pose_sample.orientation.x = 4;
  pose_sample.orientation.y = 5;
  pose_sample.orientation.z = 6;
  pose_sample.orientation.w = 7;

  ParseAndDeserialize( pose_sample, "pose");

  return 0;
}
/* Expected output

pose/position/x >> 1
pose/position/y >> 2
pose/position/z >> 3
pose/orientation/x >> 4
pose/orientation/y >> 5
pose/orientation/z >> 6
pose/orientation/w >> 7

*/
