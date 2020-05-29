#include "config.h"
#include <gtest/gtest.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16MultiArray.h>
#include "ros_msg_parser/ros_parser.hpp"

using namespace ros::message_traits;
using namespace RosMsgParser;

TEST(RosType, builtin_int32)
{
  ROSField f("int32 number");

  EXPECT_EQ(f.type().baseName(), "int32");
  EXPECT_EQ(f.type().msgName(),  "int32");
  EXPECT_EQ(f.type().pkgName().size(), 0);
  EXPECT_EQ(f.type().isBuiltin(),  true);
  EXPECT_EQ(f.type().typeSize(),  4);
  EXPECT_EQ(f.isArray(),  false);
  EXPECT_EQ(f.arraySize(),  1);
  EXPECT_EQ(f.name(), "number");
}

TEST(RosType,  builtin_string)
{
  ROSField f("string my_string");
  EXPECT_EQ(f.type().baseName(),  "string");
  EXPECT_EQ(f.type().msgName() ,  "string");
  EXPECT_EQ(f.type().pkgName().size(), 0);
  EXPECT_EQ(f.type().isBuiltin(),  true);
  EXPECT_EQ(f.type().typeSize(),  -1);
  EXPECT_EQ(f.isArray(),  false);
  EXPECT_EQ(f.arraySize(),  1);
  EXPECT_EQ(f.name(), "my_string");
}


TEST(RosType, builtin_fixedlen_array)
{
  ROSField f("float64[32] my_array");

  EXPECT_EQ(f.type().baseName(), "float64");
  EXPECT_EQ(f.type().msgName(),  "float64");
  EXPECT_EQ(f.type().pkgName().size(),  0 );
  EXPECT_EQ(f.type().isBuiltin(),  true);
  EXPECT_EQ(f.type().typeSize(),  8);
  EXPECT_EQ(f.isArray(),  true);
  EXPECT_EQ(f.arraySize(),  32);
  EXPECT_EQ(f.name(), "my_array");
}

TEST(RosType, builtin_dynamic_array)
{
  ROSField f("float32[] my_array");

  EXPECT_EQ(f.type().baseName(), "float32");
  EXPECT_EQ(f.type().msgName(),  "float32");
  EXPECT_EQ(f.type().pkgName().size(),  0 );
  EXPECT_EQ(f.type().isBuiltin(),  true);
  EXPECT_EQ(f.type().typeSize(),  4);
  EXPECT_EQ(f.isArray(),  true);
  EXPECT_EQ(f.arraySize(),  -1);
  EXPECT_EQ(f.name(), "my_array");
}


TEST(RosType, no_builtin_array)
{
  ROSField f("geometry_msgs/Pose my_pose");

  EXPECT_EQ(f.type().baseName(), "geometry_msgs/Pose" );
  EXPECT_EQ(f.type().msgName(),  "Pose" );
  EXPECT_EQ(f.type().pkgName(),  "geometry_msgs" );
  EXPECT_EQ(f.type().isBuiltin(),  false);
  EXPECT_EQ(f.type().typeSize(),  -1);
  EXPECT_EQ(f.isArray(),  false);
  EXPECT_EQ(f.arraySize(),  1);
  EXPECT_EQ(f.name(), "my_pose");
}

TEST(ROSMessageFields, ParseComments) {

  std::string
      def("MSG: geometry_msgs/Quaternion\n"
          "\n"
          "#just a comment"
          "          # I'm a comment after whitespace\n"
          "float64 x # I'm an end of line comment float64 y\n"
          "float64 z\n"
          );
  ROSMessage mt(def);
  EXPECT_EQ( mt.type().baseName(),  "geometry_msgs/Quaternion" );

  EXPECT_EQ(mt.fields().size(),  2);
  EXPECT_EQ(mt.field(0).type().msgName(),  "float64");
  EXPECT_EQ(mt.field(1).type().msgName(),  "float64");
  EXPECT_EQ(mt.field(0).name(),  "x");
  EXPECT_EQ(mt.field(1).name(),  "z");

  EXPECT_EQ( mt.field(0).isConstant(),  false);
  EXPECT_EQ( mt.field(1).isConstant(),  false);
}

TEST(ROSMessageFields, constant_uint8)
{
  ROSMessage msg("uint8 a = 66\n");

  EXPECT_EQ(msg.fields().size(),  1);
  EXPECT_EQ( msg.field(0).name(),  "a" );
  EXPECT_EQ( msg.field(0).type().baseName(),  "uint8" );
  EXPECT_EQ( msg.field(0).isConstant(),  true);
  EXPECT_EQ(msg.field(0).value(),  "66");
}

TEST(ROSMessageFields, ConstantNavstatus )
{
  ROSMessage msg( Definition<sensor_msgs::NavSatStatus >::value() );

  EXPECT_EQ( msg.fields().size(),  10);

  EXPECT_EQ( msg.field(0).name(), ("STATUS_NO_FIX") );
  EXPECT_EQ( msg.field(0).type().baseName(),  ("int8"));
  EXPECT_EQ( msg.field(0).value(), ("-1"));

  EXPECT_EQ( msg.field(1).name(),  ("STATUS_FIX") );
  EXPECT_EQ( msg.field(1).type().baseName(),  ("int8"));
  EXPECT_EQ( msg.field(1).value() ,  ("0"));

  EXPECT_EQ( msg.field(2).name(),  ("STATUS_SBAS_FIX") );
  EXPECT_EQ( msg.field(2).type().baseName(),  ("int8"));
  EXPECT_EQ( msg.field(2).value() ,  ("1"));

  EXPECT_EQ( msg.field(3).name(),  ("STATUS_GBAS_FIX") );
  EXPECT_EQ( msg.field(3).type().baseName(),  ("int8"));
  EXPECT_EQ( msg.field(3).value() ,  ("2"));

  EXPECT_EQ( msg.field(4).name(),  ("status") );
  EXPECT_EQ( msg.field(4).type().baseName(),  ("int8"));
  EXPECT_EQ( msg.field(4).isConstant() ,  false);

  EXPECT_EQ( msg.field(5).name(),  ("SERVICE_GPS") );
  EXPECT_EQ( msg.field(5).type().baseName(),  ("uint16"));
  EXPECT_EQ( msg.field(5).value() ,  ("1"));

  EXPECT_EQ( msg.field(6).name(),  ("SERVICE_GLONASS") );
  EXPECT_EQ( msg.field(6).type().baseName(),  ("uint16"));
  EXPECT_EQ( msg.field(6).value() ,  ("2"));

  EXPECT_EQ( msg.field(7).name(),  ("SERVICE_COMPASS") );
  EXPECT_EQ( msg.field(7).type().baseName(),  ("uint16"));
  EXPECT_EQ( msg.field(7).value() ,  ("4"));

  EXPECT_EQ( msg.field(8).name(),  ("SERVICE_GALILEO") );
  EXPECT_EQ( msg.field(8).type().baseName(),  ("uint16"));
  EXPECT_EQ( msg.field(8).value() ,  ("8"));

  EXPECT_EQ( msg.field(9).name(),  ("service") );
  EXPECT_EQ( msg.field(9).type().baseName(),  ("uint16"));
  EXPECT_EQ( msg.field(9).isConstant() ,  false);
}


TEST(ROSMessageFields, ConstantComments )
{
  ROSMessage msg(
        "string strA=  this string has a # comment in it  \n"
        "string strB = this string has \"quotes\" and \\slashes\\ in it\n"
        "float64 a=64.0 # numeric comment\n");


  EXPECT_EQ( msg.fields().size(),  3);
  EXPECT_EQ( ("strA"), msg.field(0).name());
  EXPECT_EQ( ("string"), msg.field(0).type().baseName() );
  EXPECT_EQ( msg.field(0).isConstant(),  true);
  EXPECT_EQ( ("this string has a # comment in it"),  msg.field(0).value());

  EXPECT_EQ( ("strB"),  msg.field(1).name());
  EXPECT_EQ( ("string"), msg.field(1).type().baseName() );
  EXPECT_EQ( msg.field(1).isConstant(),  true);
  EXPECT_EQ( ("this string has \"quotes\" and \\slashes\\ in it"),   msg.field(1).value());

  EXPECT_EQ( ("a"), msg.field(2).name());
  EXPECT_EQ( ("float64"), msg.field(2).type().baseName() );
  EXPECT_EQ( msg.field(2).isConstant(),  true);
  EXPECT_EQ( ("64.0"), msg.field(2).value());
}

/*
TEST(BuildROSTypeMapFromDefinition,  PoseParsing )
{
  RosMsgParser::Parser parser;

  parser.registerMessageDefinition(
        "pose",
        ROSType(DataType<geometry_msgs::Pose >::value()),
        Definition<geometry_msgs::Pose >::value());

  auto info = parser.getMessageInfo("pose");
  const ROSMessage* msg = &(info->type_list[0]);

  EXPECT_EQ( msg->type().baseName(),  "geometry_msgs/Pose" );
  EXPECT_EQ( msg->fields().size(),  2);
  EXPECT_EQ( msg->field(0).type().baseName(),  "geometry_msgs/Point" );
  EXPECT_EQ( msg->field(0).name(),  "position" );
  EXPECT_EQ( msg->field(1).type().baseName(),  "geometry_msgs/Quaternion" );
  EXPECT_EQ( msg->field(1).name(),  "orientation" );

  msg = &info->type_list[1];
  EXPECT_EQ( ("geometry_msgs/Point" ),  msg->type().baseName() );
  EXPECT_EQ( msg->fields().size(),  3);
  EXPECT_EQ( msg->field(0).type().baseName(),  "float64" );
  EXPECT_EQ( msg->field(0).name(),  "x" );
  EXPECT_EQ( msg->field(1).type().baseName(),  "float64" );
  EXPECT_EQ( msg->field(1).name(),  "y" );
  EXPECT_EQ( msg->field(2).type().baseName(),  "float64" );
  EXPECT_EQ( msg->field(2).name(),  "z" );

  msg = &info->type_list[2];
  EXPECT_EQ( ("geometry_msgs/Quaternion" ),  msg->type().baseName() );
  EXPECT_EQ( msg->fields().size(),  4);
  EXPECT_EQ( msg->field(0).type().baseName(),  "float64" );
  EXPECT_EQ( msg->field(0).name(),  "x" );
  EXPECT_EQ( msg->field(1).type().baseName() ,  "float64" );
  EXPECT_EQ( msg->field(1).name(),  "y" );
  EXPECT_EQ( msg->field(2).type().baseName() ,  "float64" );
  EXPECT_EQ( msg->field(2).name(),  "z" );
  EXPECT_EQ( msg->field(3).type().baseName() ,  "float64" );
  EXPECT_EQ( msg->field(3).name(),  "w" );
}
*/
TEST(BuildROSTypeMapFromDefinition,  IMUparsing )
{
  RosMsgParser::Parser parser("imu",
        ROSType(DataType<sensor_msgs::Imu >::value()),
        Definition<sensor_msgs::Imu >::value());

  auto info = parser.getMessageInfo();

  const ROSMessage* msg = &info->type_list[0];
  EXPECT_EQ( ("sensor_msgs/Imu"),  msg->type().baseName() );
  EXPECT_EQ( msg->fields().size(),  7);
  EXPECT_EQ( ("std_msgs/Header" ),  msg->field(0).type().baseName() );
  EXPECT_EQ( ("header" )         ,  msg->field(0).name() );

  EXPECT_EQ( ("geometry_msgs/Quaternion" ),  msg->field(1).type().baseName() );
  EXPECT_EQ( ("orientation" )             ,  msg->field(1).name() );

  EXPECT_EQ( ("float64" )                ,  msg->field(2).type().baseName() );
  EXPECT_EQ( ("orientation_covariance" ) ,  msg->field(2).name() );
  EXPECT_EQ( msg->field(2).arraySize(),  9);

  EXPECT_EQ( ("geometry_msgs/Vector3" ),  msg->field(3).type().baseName() );
  EXPECT_EQ( ("angular_velocity" )     ,  msg->field(3).name() );

  EXPECT_EQ( ("float64" )                    ,  msg->field(4).type().baseName() );
  EXPECT_EQ( ("angular_velocity_covariance" ),  msg->field(4).name() );
  EXPECT_EQ( msg->field(4).arraySize(),  9);

  EXPECT_EQ( ("geometry_msgs/Vector3" ),  msg->field(5).type().baseName() );
  EXPECT_EQ( ("linear_acceleration" )  ,  msg->field(5).name() );

  EXPECT_EQ( ("float64" )                       ,  msg->field(6).type().baseName() );
  EXPECT_EQ( ("linear_acceleration_covariance" ),  msg->field(6).name() );
  EXPECT_EQ( msg->field(6).arraySize(),  9);


  //---------------------------------
  msg = &info->type_list[1];
  EXPECT_EQ( msg->type().baseName(),  "std_msgs/Header" );
  EXPECT_EQ( msg->fields().size(),  3);
  EXPECT_EQ( msg->field(0).type().baseName(),  ("uint32" ));
  EXPECT_EQ( msg->field(0).name(),  ("seq") );
  EXPECT_EQ( msg->field(1).type().baseName(),  ("time") );
  EXPECT_EQ( msg->field(1).name(),  "stamp" );
  EXPECT_EQ( msg->field(2).type().baseName(),  "string" );
  EXPECT_EQ( msg->field(2).name(),  "frame_id" );

  msg = &info->type_list[2];
  EXPECT_EQ( msg->type().baseName(),  ("geometry_msgs/Quaternion") );
  EXPECT_EQ( msg->fields().size(),  4);
  EXPECT_EQ( msg->field(0).type().baseName(),  "float64" );
  EXPECT_EQ( msg->field(0).name(),  "x" );
  EXPECT_EQ( msg->field(1).type().baseName(),  "float64" );
  EXPECT_EQ( msg->field(1).name(),  "y" );
  EXPECT_EQ( msg->field(2).type().baseName(),  "float64" );
  EXPECT_EQ( msg->field(2).name(),  "z" );
  EXPECT_EQ( msg->field(3).type().baseName(),  "float64" );
  EXPECT_EQ( msg->field(3).name(),  "w" );

  msg = &info->type_list[3];
  EXPECT_EQ( msg->type().baseName(),  ("geometry_msgs/Vector3") );
  EXPECT_EQ( msg->fields().size(),  3);
  EXPECT_EQ( msg->field(0).type().baseName(),  "float64" );
  EXPECT_EQ( msg->field(0).name(),  "x" );
  EXPECT_EQ( msg->field(1).type().baseName(),  "float64" );
  EXPECT_EQ( msg->field(1).name(),  "y" );
  EXPECT_EQ( msg->field(2).type().baseName(),  "float64" );
  EXPECT_EQ( msg->field(2).name(),  "z" );
}

TEST(BuildROSTypeMapFromDefinition,  Int16MultiArrayParsing )
{
  // this test case was added because it previously failed to detect nested
  // arrays of custom types. In this case:
  //    std_msgs/MultiArrayDimension[]

  RosMsgParser::Parser parser( "multiarray",
        ROSType(DataType<std_msgs::Int16MultiArray >::value()),
        Definition<std_msgs::Int16MultiArray >::value());

  /*std_msgs/Int16MultiArray :
          layout : std_msgs/MultiArrayLayout
          data : int16[]

      std_msgs/MultiArrayLayout :
          dim : std_msgs/MultiArrayDimension[]
          data_offset : uint32

      std_msgs/MultiArrayDimension :
          label : string
          size : uint32
          stride : uint32*/

  auto info = parser.getMessageInfo();
  const ROSMessage* msg = &info->type_list[0];

  EXPECT_EQ( ("std_msgs/Int16MultiArray"),  msg->type().baseName() );
  EXPECT_EQ( msg->fields().size(),  2);
  EXPECT_EQ( ("std_msgs/MultiArrayLayout" ),  msg->field(0).type().baseName() );
  EXPECT_EQ( ("layout" )                   ,  msg->field(0).name() );
  EXPECT_EQ( false,  msg->field(0).isArray() );

  EXPECT_EQ( ("int16" ),  msg->field(1).type().baseName() );
  EXPECT_EQ( ("data" ) ,  msg->field(1).name() );
  EXPECT_EQ( true,  msg->field(1).isArray() );

  msg = &info->type_list[1];
  EXPECT_EQ( ("std_msgs/MultiArrayLayout"),  msg->type().baseName() );
  EXPECT_EQ( msg->fields().size(),  2);
  EXPECT_EQ( ("std_msgs/MultiArrayDimension" ),    msg->field(0).type().baseName() );
  EXPECT_EQ( ("dim" )                           ,  msg->field(0).name() );
  EXPECT_EQ( true,  msg->field(0).isArray() );

  EXPECT_EQ( ("uint32" )      ,  msg->field(1).type().baseName() );
  EXPECT_EQ( ("data_offset" ) ,  msg->field(1).name() );
  EXPECT_EQ( false,  msg->field(1).isArray() );


  msg = &info->type_list[2];
  EXPECT_EQ( ("std_msgs/MultiArrayDimension"),  msg->type().baseName() );
  EXPECT_EQ( msg->fields().size(),  3);

  EXPECT_EQ( ("string" ),  msg->field(0).type().baseName() );
  EXPECT_EQ( ("label" )  ,  msg->field(0).name() );
  EXPECT_EQ( false,  msg->field(0).isArray() );

  EXPECT_EQ( ("uint32" ),  msg->field(1).type().baseName() );
  EXPECT_EQ( ("size" )  ,  msg->field(1).name() );
  EXPECT_EQ( false,  msg->field(1).isArray() );

  EXPECT_EQ( ("uint32" ) ,  msg->field(2).type().baseName() );
  EXPECT_EQ( ("stride" ) ,  msg->field(2).name() );
  EXPECT_EQ( false,  msg->field(2).isArray() );
}



