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

#include "ros_msg_parser/stringtree_leaf.hpp"
#include "ros_msg_parser/helper_functions.hpp"

namespace RosMsgParser{


bool FieldTreeLeaf::toStr(std::string& out) const
{
  const FieldTreeNode* leaf_node = this->node_ptr;
  if( !leaf_node ){
    return false;
  }

  size_t total_size = 0;
  boost::container::static_vector<const ROSField*, 16> field_chain;

  while(leaf_node)
  {
    const ROSField* field = leaf_node->value();
    field_chain.push_back( field );

    total_size += field->name().size() + 1;
    if (field->isArray())
    {
      total_size += 5;  // super conservative (9999)
    }
    leaf_node = leaf_node->parent();
  };

  out.resize( total_size );
  char* buffer = static_cast<char*>(&out[0]);
  std::reverse(field_chain.begin(),  field_chain.end() );

  size_t array_count = 0;
  size_t offset = 0;

  for(  const ROSField* field: field_chain)
  {
    const std::string& str = field->name();
    bool is_root = ( field == field_chain.front() );
    if( !is_root ){
      buffer[offset++] = '/';
    }
    std::memcpy( &buffer[offset], str.data(), str.size() );
    offset += str.size();

    if(!is_root && field->isArray())
    {
      buffer[offset++] = '.';
      offset += print_number(&buffer[offset], index_array[ array_count++ ] );
    }
  }
  buffer[offset] = '\0';
  out.resize(offset);

  return true;
}


void CreateStringFromTreeLeaf(const FieldTreeLeaf &leaf, bool skip_root, std::string& out)
{
  const FieldTreeNode* leaf_node = leaf.node_ptr;
  if( !leaf_node ){
      out.clear();
      return ;
  }

  boost::container::static_vector<const std::string*, 16> strings_chain;

  size_t total_size = 0;

  while(leaf_node)
  {
    const auto& str = leaf_node->value()->name();
    leaf_node = leaf_node->parent();
    if( !( leaf_node == nullptr && skip_root) )
    {
        strings_chain.emplace_back( &str );
        const size_t S = str.size();
        if( S == 1 && str[0] == '#' )
        {
            total_size += 5; // super conservative
        }
        else{
          total_size += S+1;
        }
    }
  };

  out.resize(total_size);
  char* buffer = &out[0];

  std::reverse(strings_chain.begin(),  strings_chain.end() );

  size_t array_count = 0;
  size_t offset = 0;

  for( const auto& str: strings_chain)
  {
    const size_t S = str->size();
    if( S == 1 && (*str)[0] == '#' )
    {
      buffer[offset++] = '.';
      offset += print_number(&buffer[offset], leaf.index_array[ array_count++ ] );
    }
    else{
      if( str !=  strings_chain.front() ){
        buffer[offset++] = '/';
      }
      std::memcpy( &buffer[offset], str->data(), S );
      offset += S;
    }
  }
  out.resize(offset);
}

}
