#include <ros_msg_parser/ros_parser.hpp>
#include <geometry_msgs/TransformStamped.h>

int main()
{
    using geometry_msgs::TransformStamped;
    using namespace RosMsgParser;

    const std::string topic_name = "transform_stamped";

    RosMsgParser::Parser parser(topic_name,
                                ROSType(ros::message_traits::DataType<TransformStamped>::value()),
                                ros::message_traits::Definition<TransformStamped>::value());

    // Let's create a random one
    TransformStamped tr;
    tr.header.seq = 42;
    tr.header.frame_id = "this_one";

    tr.transform.translation.x = 1;
    tr.transform.translation.y = 2;
    tr.transform.translation.z = 3;

    tr.transform.rotation.x = 0;
    tr.transform.rotation.y = 0;
    tr.transform.rotation.z = 0;
    tr.transform.rotation.w = 1;

    // Serialize to buffer
    std::vector<uint8_t> raw_buffer( ros::serialization::serializationLength(tr) );
    ros::serialization::OStream stream(raw_buffer.data(), raw_buffer.size());
    ros::serialization::Serializer<TransformStamped>::write(stream, tr);

    // Deserialize only the header
    std_msgs::Header header;

    const RosMsgParser::Parser::VisitingCallback modifyTimestamp =
            [&header](const RosMsgParser::ROSType&, Span<uint8_t>& buffer)
    {
        ros::serialization::IStream is( buffer.data(), buffer.size() );
        ros::serialization::deserialize(is, header);
    };

    const RosMsgParser::ROSType header_type( ros::message_traits::DataType<std_msgs::Header>::value() ) ;
    Span<uint8_t> buffer_span(raw_buffer);
    parser.applyVisitorToBuffer(header_type, buffer_span,  modifyTimestamp );

    std::cout << "Seq: "   << header.seq << std::endl;
    std::cout << "Frame: " << header.frame_id << std::endl;

    return 0;
}

