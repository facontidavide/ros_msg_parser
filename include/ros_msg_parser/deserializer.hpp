#ifndef DESERIALIZER_HPP
#define DESERIALIZER_HPP

// API adapted to FastCDR

#include <exception>
#include "ros_msg_parser/builtin_types.hpp"
#include "ros_msg_parser/variant.hpp"

namespace RosMsgParser
{

class Deserializer
{
public:
  Deserializer( Span<uint8_t> buffer ):
    _buffer(buffer)
  {}

  virtual ~Deserializer() = default;

  // move the memory pointer
  virtual void jump(size_t bytes) = 0;

  // deserialize the current pointer into a variant (not a string)
  virtual Variant deserialize(BuiltinType type) = 0;

  // deserialize the current pointer into a string
  virtual void deserializeString(std::string& out) = 0;

  virtual const uint8_t* getCurrentPtr() const = 0;

  // reset the pointer to beginning of buffer
  virtual void reset() = 0;

protected:
  Span<uint8_t> _buffer;
};

// Sopecialization od deserializer that works with ROS1
class ROS_Deserializer : public Deserializer
{
public:
  ROS_Deserializer( Span<uint8_t> buffer );

  virtual Variant deserialize(BuiltinType type) override;

  virtual void deserializeString(std::string& dst) override;

  void jump(size_t bytes) override;

  virtual void reset() override;

protected:

  const uint8_t* _ptr;
  size_t _bytes_left;

  template <typename T> T deserialize()
  {
    T out;
    if ( sizeof(T) > _bytes_left )
    {
      throw std::runtime_error("Buffer overrun in Deserializer");
    }
    out =  ( *(reinterpret_cast<const T*>( _ptr )) );
    _bytes_left -= sizeof(T);
    _ptr += sizeof(T);
    return out;
  }
};

}

#endif // DESERIALIZER_HPP
