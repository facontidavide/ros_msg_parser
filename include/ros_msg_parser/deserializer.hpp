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

  virtual void init(Span<const uint8_t> buffer)
  {
    _buffer = buffer;
    reset();
  }

  virtual ~Deserializer() = default;

  // move the memory pointer
  virtual void jump(size_t bytes) = 0;

  // deserialize the current pointer into a variant (not a string)
  virtual Variant deserialize(BuiltinType type) = 0;

  // deserialize the current pointer into a string
  virtual void deserializeString(std::string& out) = 0;

  virtual uint32_t deserializeUInt32() = 0;

  virtual const uint8_t* getCurrentPtr() const = 0;

  virtual size_t bytesLeft() const
  {
    return _buffer.size() - ( getCurrentPtr() - _buffer.data() );
  }

  // reset the pointer to beginning of buffer
  virtual void reset() = 0;

protected:
  Span<const uint8_t> _buffer;
};

// Sopecialization od deserializer that works with ROS1
class ROS_Deserializer : public Deserializer
{
public:

  virtual Variant deserialize(BuiltinType type) override;

  virtual void deserializeString(std::string& dst) override;

  virtual uint32_t deserializeUInt32() override;

  virtual const uint8_t* getCurrentPtr() const override;

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
