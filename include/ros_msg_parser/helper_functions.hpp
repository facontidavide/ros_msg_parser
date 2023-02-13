/***** MIT License ****
*
*   Copyright (c) 2016-2022 Davide Faconti
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy
*   of this software and associated documentation files (the "Software"), to deal
*   in the Software without restriction, including without limitation the rights
*   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*   copies of the Software, and to permit persons to whom the Software is
*   furnished to do so, subject to the following conditions:
*
*   The above copyright notice and this permission notice shall be included in all
*    copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*   SOFTWARE.
*/

#ifndef ROS_INTROSPECTION_HELPER_H
#define ROS_INTROSPECTION_HELPER_H

#include <functional>
#include <ros_msg_parser/utils/variant.hpp>
#include <ros_msg_parser/utils/span.hpp>

namespace RosMsgParser{

template< class T>
using Span = nonstd::span<T>;

// Brutally faster for numbers below 100
inline int print_number(char* buffer, uint16_t value)
{
  const char DIGITS[] =
      "00010203040506070809"
      "10111213141516171819"
      "20212223242526272829"
      "30313233343536373839"
      "40414243444546474849"
      "50515253545556575859"
      "60616263646566676869"
      "70717273747576777879"
      "80818283848586878889"
      "90919293949596979899";
  if (value < 10)
  {
    buffer[0] = static_cast<char>('0' + value);
    return 1;
  }
  else if (value < 100) {
    value *= 2;
    buffer[0] = DIGITS[ value ];
    buffer[1] = DIGITS[ value+1 ];
    return 2;
  }
  else{
    return sprintf( buffer,"%d", value );
  }
}


// helper function to deserialize raw memory
template <typename T> inline void ReadFromBuffer( const Span<const uint8_t>& buffer, size_t& offset, T& destination)
{
  if ( offset + sizeof(T) > static_cast<std::size_t>(buffer.size()) )
  {
    throw std::runtime_error("Buffer overrun in RosMsgParser::ReadFromBuffer");
  }
  destination =  (*( reinterpret_cast<const T*>( &(buffer.data()[offset]) ) ) );
  offset += sizeof(T);
}

template <> inline void ReadFromBuffer( const Span<const uint8_t>& buffer, size_t& offset, std::string& destination)
{
  uint32_t string_size = 0;
  ReadFromBuffer( buffer, offset, string_size );

  if( offset + string_size > static_cast<std::size_t>(buffer.size()) )
  {
    throw std::runtime_error("Buffer overrun in RosMsgParser::ReadFromBuffer");
  }

  if (string_size == 0) {
    destination = std::string();
    return;
  }

  const char* buffer_ptr = reinterpret_cast<const char*>( &buffer[offset] );
  offset += string_size;

  destination.assign( buffer_ptr, string_size );
}

template <typename T> inline T ReadFromBuffer( const Span<const uint8_t>& buffer, size_t& offset)
{
  T out;
  ReadFromBuffer(buffer, offset, out);
  return out;
}

template <typename T> inline
Variant ReadFromBufferToVariant( const Span<const uint8_t>& buffer, size_t& offset)
{
  T destination;
  ReadFromBuffer(buffer, offset, destination);
  return Variant(destination);
}

inline Variant ReadFromBufferToVariant(BuiltinType id, const Span<const uint8_t>& buffer, size_t& offset)
{
  switch(id)
  {
  case BOOL: return ReadFromBufferToVariant<bool>(buffer,offset);
  case CHAR: return ReadFromBufferToVariant<char>(buffer,offset);
  case BYTE:
  case UINT8:  return ReadFromBufferToVariant<uint8_t>(buffer,offset);
  case UINT16: return ReadFromBufferToVariant<uint16_t>(buffer,offset);
  case UINT32: return ReadFromBufferToVariant<uint32_t>(buffer,offset);
  case UINT64: return ReadFromBufferToVariant<uint64_t>(buffer,offset);

  case INT8:   return ReadFromBufferToVariant<int8_t>(buffer,offset);
  case INT16:  return ReadFromBufferToVariant<int16_t>(buffer,offset);
  case INT32:  return ReadFromBufferToVariant<int32_t>(buffer,offset);
  case INT64:  return ReadFromBufferToVariant<int64_t>(buffer,offset);

  case FLOAT32:  return ReadFromBufferToVariant<float>(buffer,offset);
  case FLOAT64:  return ReadFromBufferToVariant<double>(buffer,offset);

  case TIME: {
    ros::Time tmp;
    ReadFromBuffer( buffer, offset, tmp.sec );
    ReadFromBuffer( buffer, offset, tmp.nsec );
    return tmp;
  }
  case DURATION: {
    ros::Duration tmp;
    ReadFromBuffer( buffer, offset, tmp.sec );
    ReadFromBuffer( buffer, offset, tmp.nsec );
    return tmp;
  }

  case STRING: {
    uint32_t string_size = 0;
    ReadFromBuffer( buffer, offset, string_size );
    if( offset + string_size > static_cast<std::size_t>(buffer.size()) ) {
      throw std::runtime_error("Buffer overrun");
    }
    Variant var_string(reinterpret_cast<const char*>( &buffer[offset] ), string_size  );
    offset += string_size;
    return var_string;
  }
  case OTHER: return -1;
  default: break;
  }
  throw std::runtime_error( "unsupported builtin type value");
}


} // end namespace


#endif
