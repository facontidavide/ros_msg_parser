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

#ifndef ROS_INTROSPECTION_FieldTreeLeaf_H
#define ROS_INTROSPECTION_FieldTreeLeaf_H

#include <vector>
#include <map>
#include <iostream>
#include <boost/container/small_vector.hpp>
#include <boost/container/static_vector.hpp>
#include "ros_msg_parser/ros_message.hpp"

namespace RosMsgParser{

/**
 * @brief The FieldTreeLeaf is, as the name suggests, a leaf (terminal node)
 * of a StringTree.
 * It provides the pointer to the node and a list of numbers that represent
 * the index that corresponds to the placeholder "#".
 *
 * For example if you want to represent the string
 *
 *      foo/2/bar/3/hello/world
 *
 * This would correspond to a branch of the tree (from root to the leaf) equal to these 6 nodes,
 * where "foo" is the root and "world" is the leaf
 *
 * foo -> # -> bar -> # ->hello -> world
 *
 * array_size will be equal to two and index_array will contain these numbers {2,3}
 *
 */
struct FieldTreeLeaf{

  FieldTreeLeaf();

  const FieldTreeNode* node_ptr;

  boost::container::static_vector<uint16_t,8> index_array;

  /// Utility functions to print the entire branch
  bool toStr(std::string &destination) const;

  std::string toStdString() const
  {
    std::string out;
    toStr(out);
    return out;
  }
};

void CreateStringFromTreeLeaf(const FieldTreeLeaf& leaf, bool skip_root, std::string &out);

//---------------------------------

inline std::ostream& operator<<(std::ostream &os, const FieldTreeLeaf& leaf )
{
  std::string dest;
  leaf.toStr(dest);
  os << dest;
  return os;
}

inline FieldTreeLeaf::FieldTreeLeaf(): node_ptr(nullptr)
{  }



}

#endif // ROSTYPE_H
