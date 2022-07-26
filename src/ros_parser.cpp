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
*   copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*   SOFTWARE.
*/

#include <functional>

#include "ros_msg_parser/ros_parser.hpp"
#include "ros_msg_parser/deserializer.hpp"

namespace RosMsgParser
{
inline bool operator==(const std::string& a, const std::string_view& b)
{
  return (a.size() == b.size() && std::strncmp(a.data(), b.data(), a.size()) == 0);
}

Parser::Parser(const std::string &topic_name,
               const ROSType &msg_type,
               const std::string &definition)
  : _global_warnings(&std::cerr)
  , _topic_name(topic_name)
  , _msg_type(msg_type)
  , _discard_large_array(DISCARD_LARGE_ARRAYS)
  , _max_array_size(100)
  , _blob_policy(STORE_BLOB_AS_COPY)
  , _dummy_root_field( new ROSField(_msg_type, topic_name) )
{
  auto parsed_msgs = ParseMessageDefinitions(definition, msg_type);
  _schema = BuildMessageSchema(topic_name, parsed_msgs);
}

const std::shared_ptr<MessageSchema>& Parser::getSchema() const
{
  return _schema;
}

ROSMessage::Ptr Parser::getMessageByType(const ROSType& type) const
{
  for (const auto& [msg_type, msg] : _schema->msg_library)  // find in the list
  {
    if (msg_type == type)
    {
      return msg;
    }
  }
  return nullptr;
}

template <typename Container>
inline void ExpandVectorIfNecessary(Container& container, size_t new_size)
{
  if (container.size() <= new_size)
  {
    const size_t increased_size = std::max(size_t(32), container.size() * 2);
    container.resize(increased_size);
  }
}

bool Parser::deserialize(Span<const uint8_t> buffer,
                         FlatMessage* flat_container) const
{

 /* bool entire_message_parse = true;

  size_t value_index = 0;
  size_t name_index = 0;
  size_t blob_index = 0;
  size_t blob_storage_index = 0;

  size_t buffer_offset = 0;

  std::function<void(const ROSMessage*, FieldTreeLeaf, bool)> deserializeImpl;

  deserializeImpl = [&](const ROSMessage* msg, FieldTreeLeaf tree_leaf, bool store)
  {
    size_t index_s = 0;
    size_t index_m = 0;

    for (const ROSField& field : msg->fields())
    {
      bool DO_STORE = store;
      if (field.isConstant())
      {
        continue;
      }

      const ROSType& field_type = field.type();

      auto new_tree_leaf = tree_leaf;
      new_tree_leaf.node_ptr = tree_leaf.node_ptr->child(index_s);

      int32_t array_size = field.arraySize();
      if (array_size == -1)
      {
        ReadFromBuffer(buffer, buffer_offset, array_size);
      }
      if (field.isArray())
      {
        new_tree_leaf.index_array.push_back(0);
      }

      bool IS_BLOB = false;

      // Stop storing it if is NOT a blob and a very large array.
      if (array_size > static_cast<int32_t>(_max_array_size) &&
          field_type.typeID() == BuiltinType::OTHER)
      {
        if (builtinSize(field_type.typeID()) == 1)
        {
          IS_BLOB = true;
        }
        else
        {
          if (_discard_large_array)
          {
            DO_STORE = false;
          }
          entire_message_parse = false;
        }
      }

      if (IS_BLOB)  // special case. This is a "blob", typically an image, a map, pointcloud, etc.
      {
        ExpandVectorIfNecessary(flat_container->blob, blob_index);

        if (buffer_offset + array_size > static_cast<std::size_t>(buffer.size()))
        {
          throw std::runtime_error("Buffer overrun in deserializeIntoFlatContainer (blob)");
        }
        if (DO_STORE)
        {
          flat_container->blob[blob_index].first = new_tree_leaf;
          auto& blob = flat_container->blob[blob_index].second;
          blob_index++;

          if (_blob_policy == STORE_BLOB_AS_COPY)
          {
            ExpandVectorIfNecessary(flat_container->blob_storage, blob_storage_index);

            auto& storage = flat_container->blob_storage[blob_storage_index];
            storage.resize(array_size);
            std::memcpy(storage.data(), &buffer[buffer_offset], array_size);
            blob_storage_index++;

            blob = Span<const uint8_t>(storage.data(), storage.size());
          }
          else
          {
            blob = Span<const uint8_t>(&buffer[buffer_offset], array_size);
          }
        }
        buffer_offset += array_size;
      }
      else  // NOT a BLOB
      {
        bool DO_STORE_ARRAY = DO_STORE;
        for (int i = 0; i < array_size; i++)
        {
          if (DO_STORE_ARRAY && i >= static_cast<int32_t>(_max_array_size))
          {
            DO_STORE_ARRAY = false;
          }

          if (field.isArray() && DO_STORE_ARRAY)
          {
            new_tree_leaf.index_array.back() = i;
          }

          if (field_type.typeID() == STRING)
          {
            ExpandVectorIfNecessary(flat_container->name, name_index);

            uint32_t string_size = 0;
            ReadFromBuffer(buffer, buffer_offset, string_size);

            if (buffer_offset + string_size > static_cast<std::size_t>(buffer.size()))
            {
              throw std::runtime_error("Buffer overrun in RosMsgParser::ReadFromBuffer");
            }

            if (DO_STORE_ARRAY)
            {
              if (string_size == 0)
              {
                // corner case, when there is an empty string at the end of the message
                flat_container->name[name_index].second.clear();
              }
              else
              {
                const char* buffer_ptr = reinterpret_cast<const char*>(buffer.data() + buffer_offset);
                flat_container->name[name_index].second.assign(buffer_ptr, string_size);
              }
              flat_container->name[name_index].first = new_tree_leaf;
              name_index++;
            }
            buffer_offset += string_size;
          }
          else if (field_type.isBuiltin())
          {
            ExpandVectorIfNecessary(flat_container->value, value_index);

            Variant var = ReadFromBufferToVariant(field_type.typeID(), buffer, buffer_offset);
            if (DO_STORE_ARRAY)
            {
              flat_container->value[value_index] = std::make_pair(new_tree_leaf, std::move(var));
              value_index++;
            }
          }
          else
          {  // field_type.typeID() == OTHER

            deserializeImpl(msg_node->child(index_m), new_tree_leaf, DO_STORE_ARRAY);
          }
        }  // end for array_size
      }

      if (field_type.typeID() == OTHER)
      {
        index_m++;
      }
      index_s++;
    }  // end for fields
  };   // end of lambda

  // pass the shared_ptr
  flat_container->schema = _schema;

  FieldTreeLeaf rootnode;
  rootnode.node_ptr = _schema->field_tree.croot();

  deserializeImpl(_schema->field_tree.croot(), rootnode, true);

  flat_container->name.resize(name_index);
  flat_container->value.resize(value_index);
  flat_container->blob.resize(blob_index);
  flat_container->blob_storage.resize(blob_storage_index);

  if (buffer_offset != static_cast<std::size_t>(buffer.size()))
  {
    char msg_buff[1000];
    sprintf(msg_buff,
            "buildRosFlatType: There was an error parsing the buffer.\n"
            "Size %d != %d, while parsing [%s]",
            (int)buffer_offset, (int)buffer.size(), _topic_name.c_str());

    throw std::runtime_error(msg_buff);
  }
  return entire_message_parse;*/
 return true;
}


void CreateRenamedValues(const FlatMessage& flat_msg, RenamedValues& renamed)
{
/*  renamed.resize(flat_msg.value.size());
  for (size_t i = 0; i < flat_msg.value.size(); i++)
  {
    const auto& in = flat_msg.value[i];
    auto& out = renamed[i];
    in.first.toStr(out.first);
    out.second = in.second.convert<double>();
  }*/
}

void ParsersCollection::registerParser(const std::string& topic_name, const ROSType& msg_type,
                                       const std::string& definition)
{
  auto it = _pack.find(topic_name);
  if (it == _pack.end())
  {
    Parser parser(topic_name, msg_type, definition);
    CachedPack pack = { std::move(parser), {} };
    _pack.insert({ topic_name, std::move(pack) });
  }
}

const Parser* ParsersCollection::getParser(const std::string& topic_name) const
{
  auto it = _pack.find(topic_name);
  if (it != _pack.end())
  {
    return &it->second.parser;
  }
  return nullptr;
}

const ParsersCollection::DeserializedMsg* ParsersCollection::deserialize(const std::string& topic_name,
                                                                         Span<const uint8_t> buffer)
{
  auto it = _pack.find(topic_name);
  if (it != _pack.end())
  {
    CachedPack& pack = it->second;
    Parser& parser = pack.parser;
    FlatMessage& flat_msg = pack.msg.flat_msg;
    RenamedValues& renamed = pack.msg.renamed_vals;

    parser.deserialize(buffer, &flat_msg);
    CreateRenamedValues(flat_msg, renamed);

    return &pack.msg;
  }
  return nullptr;
}

}  // namespace RosMsgParser
