#include "ros_msg_parser/ros_parser.hpp"
#include <ros/ros.h>

void topicCallback(const RosMsgParser::ShapeShifter& msg,
                   const std::string& topic_name,
                   RosMsgParser::ParsersCollection& parsers)
{
  // you must register the topic definition.
  //  Don't worry, it will not be done twice
  parsers.registerParser(topic_name, msg);

  auto deserialized_msg = parsers.deserialize(topic_name, msg);

  // Print the content of the message
  printf("--------- %s ----------\n", topic_name.c_str());
  for (auto it : deserialized_msg->renamed_vals)
  {
    const std::string& key = it.first;
    const double value = it.second;
    printf(" %s = %f\n", key.c_str(), value);
  }
  for (auto it : deserialized_msg->flat_msg.name)
  {
    const std::string& key = it.first.toStdString();
    const std::string& value = it.second;
    printf(" %s = %s\n", key.c_str(), value.c_str());
  }
}

// usage: pass the name of the topic as command line argument
int main(int argc, char** argv)
{
  RosMsgParser::ParsersCollection parsers;

  if (argc != 2)
  {
    printf("Usage: rosbag_example topic_name\n");
    return 1;
  }
  const std::string topic_name = argv[1];

  ros::init(argc, argv, "universal_subscriber");
  ros::NodeHandle nh;

  // who is afraid of lambdas and boost::functions ?
  boost::function<void(const RosMsgParser::ShapeShifter::ConstPtr&)> callback;
  callback = [&parsers, topic_name](const RosMsgParser::ShapeShifter::ConstPtr& msg) -> void {
    topicCallback(*msg, topic_name, parsers);
  };
  ros::Subscriber subscriber = nh.subscribe(topic_name, 10, callback);

  ros::spin();
  return 0;
}
