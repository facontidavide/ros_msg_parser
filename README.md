# Ros Message Parser

This is the successor of [ros_type_introspection](https://github.com/facontidavide/ros_type_introspection),
just simpler and easier to use.

This C++ library extracts information from a ROS Message, even when its
type is **unknown** at compilation time.

Have you ever wanted to build an app that can subscribe to __any__ 
`topic` and extract its content, or can read data from __any__ `rosbag`? 
What if the topic and/or the bag contains user defined ROS types ignored 
at compilation time?

The common solution in the ROS ecosystem is to use Python, that provides
the needed introspection. Tools, for instance, like __rqt_plot__, __rqt_bag__ and __rosbridge__
took this approach. This library implements a __C++ alternative__.

This library is particularly useful to extract data from two type-erasing classes 
provided by ROS itself:

1. [topic_tools::ShapeShifter](http://docs.ros.org/diamondback/api/topic_tools/html/classtopic__tools_1_1ShapeShifter.html):
a type used to subscribe to any topic, regardless of the original type.

2. [rosbag::MessageInstance](http://docs.ros.org/diamondback/api/rosbag/html/c++/classrosbag_1_1MessageInstance.html):
the generic type commonly used to read data from a ROS bag.

# Examples

## Generic Topic Subscriber

```C++
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
    double value = it.second;
    std::cout << key << " = " << value << std::endl;
  }
  for (auto it : deserialized_msg->flat_msg.name)
  {
    const std::string& key = it.first.toStdString();
    const std::string& value = it.second;
    std::cout << key << " = " << value << std::endl;
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
```

## Generic rosbag reader

```C++
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
      std::cout << key << " = " << value << std::endl;
    }
    for (const auto& it : deserialized_msg->flat_msg.name)
    {
      const std::string& key = it.first.toStdString();
      const std::string& value = it.second;
      std::cout << key << " = " << value << std::endl;
    }
  }
  return 0;
}
```

# Acknowledgements

This library is inspired by these other libraries 
[matlab_rosbag](https://github.com/bcharrow/matlab_rosbag) and 
[cpp_introspection](https://github.com/tu-darmstadt-ros-pkg/cpp_introspection).


You might want to have a look to [ros_babel_fish](https://github.com/StefanFabian/ros_babel_fish).

   




 



 
