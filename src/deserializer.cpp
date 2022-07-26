#include "ros_msg_parser/deserializer.hpp"

namespace RosMsgParser
{

ROS_Deserializer::ROS_Deserializer(Span<uint8_t> buffer):
  Deserializer(buffer)
{
  reset();
}

Variant ROS_Deserializer::deserialize(BuiltinType type)
{
  switch(type)
  {
    case BOOL: return deserialize<bool>();
    case CHAR: return deserialize<char>();
    case BYTE:
    case UINT8:  return deserialize<uint8_t>();
    case UINT16: return deserialize<uint16_t>();
    case UINT32: return deserialize<uint32_t>();
    case UINT64: return deserialize<uint64_t>();

    case INT8:   return deserialize<int8_t>();
    case INT16:  return deserialize<int16_t>();
    case INT32:  return deserialize<int32_t>();
    case INT64:  return deserialize<int64_t>();

    case FLOAT32:  return deserialize<float>();
    case FLOAT64:  return deserialize<double>();

    case DURATION:
    case TIME: {
      RosMsgParser::Time tmp;
      tmp.sec = deserialize<uint32_t>();
      tmp.nsec = deserialize<uint32_t>();
      return tmp;
    }

    default:
      std::runtime_error("ROS_Deserializer: type not recognized");
  }

  return {};
}

void ROS_Deserializer::deserializeString(std::string &dst)
{
  uint32_t string_size = deserialize<uint32_t>();

  if( string_size > _bytes_left )
  {
    throw std::runtime_error("Buffer overrun in RosMsgParser::ReadFromBuffer");
  }

  if (string_size == 0) {
    dst = {};
    return;
  }

  const char* buffer_ptr = reinterpret_cast<const char*>( _ptr );
  dst.assign( buffer_ptr, string_size );

  _ptr += string_size;
  _bytes_left -= string_size;
}

void ROS_Deserializer::jump(size_t bytes)
{
  if( bytes > _bytes_left )
  {
    throw std::runtime_error("Buffer overrun");
  }
  _ptr += bytes;
  _bytes_left -= bytes;
}

void ROS_Deserializer::reset()
{
  _ptr = _buffer.data();
  _bytes_left = _buffer.size();
}

}
