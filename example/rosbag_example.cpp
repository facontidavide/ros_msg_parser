#include "ros_msg_parser/ros_parser.hpp"
#include <ros/ros.h>


// usage: pass the name of the file as command line argument
int main(int argc, char** argv)
{
  if (argc != 2)
  {
    printf("Usage: pass the name of a file as first argument\n");
    return 1;
  }

  RosMsgParser::ParsersCollection parsers;

  rosbag::Bag bag;
  try
  {
    bag.open(argv[1]);
  }
  catch (rosbag::BagException& ex)
  {
    printf("rosbag::open thrown an exception: %s\n", ex.what());
    return -1;
  }

  // this  rosbag::View will accept ALL the messages
  rosbag::View bag_view(bag);

  // register (only once at the beginning) the type of messages
  for (const rosbag::ConnectionInfo* connection : bag_view.getConnections())
  {
    const std::string& topic_name = connection->topic;
    parsers.registerParser(topic_name, *connection);
  }

  for (rosbag::MessageInstance msg_instance : bag_view)
  {
    const std::string& topic_name = msg_instance.getTopic();
    const auto deserialized_msg = parsers.deserialize(topic_name, msg_instance);

    // Print the content of the message
    printf("--------- %s ----------\n", topic_name.c_str());
    for (const auto& it : deserialized_msg->renamed_vals)
    {
      const std::string& key = it.first;
      const double value = it.second;
      printf(" %s = %f\n", key.c_str(), value);
    }
    for (const auto& it : deserialized_msg->flat_msg.name)
    {
      const std::string& key = it.first.toStdString();
      const std::string& value = it.second;
      printf(" %s = %s\n", key.c_str(), value.c_str());
    }
  }
  return 0;
}
