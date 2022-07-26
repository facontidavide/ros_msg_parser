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

#include <string_view>
#include <sstream>
#include <regex>
#include "ros_msg_parser/ros_message.hpp"

namespace RosMsgParser{

ROSMessage::ROSMessage(const std::string &msg_def)
{
  std::istringstream messageDescriptor(msg_def);
  std::match_results<std::string::const_iterator> what;

  for (std::string line; std::getline(messageDescriptor, line, '\n') ; )
  {
    std::string::const_iterator begin = line.begin(), end = line.end();

    // Skip empty line or one that is a comment
    if (std::regex_search( begin, end, what,
                           std::regex("(^\\s*$|^\\s*#)")))
    {
      continue;
    }

    // Trim start of line
    line.erase(line.begin(), std::find_if(line.begin(), line.end(),
      std::not1(std::ptr_fun<int, int>(std::isspace))));

    if( line.compare(0, 5, "MSG: ") == 0)
    {
      line.erase(0,5);
      _type = ROSType(line);
    }
    else{
      auto new_field = ROSField(line);
      _fields.push_back(new_field);
    }
  }
}

std::vector<std::string> SplitMultipleMessageDefinitions(const std::string &multi_def)
{
  std::istringstream ss_msg(multi_def);

  std::vector<std::string> parts;
  std::string part;

  for (std::string line; std::getline(ss_msg, line, '\n') ; )
  {
    if( line.find("========") == 0)
    {
      parts.emplace_back( std::move(part) );
      part = {};
    }
    else{
      part.append(line);
      part.append("\n");
    }
  }
  parts.emplace_back( std::move(part) );

  return parts;
}

void AddMessageDefinitionsToLibrary(const std::string& multi_def,
                                    RosMessageLibrary& library,
                                    const std::string& type_name )
{
  auto parts = SplitMultipleMessageDefinitions(multi_def);
  std::vector<ROSType> all_types;
  std::vector<ROSMessage::Ptr> parsed_msgs;

  for( int i = parts.size()-1; i>=0; i--)
  {
    auto msg = std::make_shared<ROSMessage>(parts[i]);
    if( i == 0 && !type_name.empty() )
    {
      msg->setType( ROSType(type_name) );
    }

    parsed_msgs.push_back( msg );
    all_types.push_back( msg->type() );
  }

  // adjust types with undefined package type
  for(const auto& msg: parsed_msgs)
  {
    for (ROSField& field: msg->fields())
    {
      // if package name is missing, try to find msgName in the list of known_type
      if( field.type().pkgName().empty() )
      {
        for (const ROSType& known_type: all_types)
        {
          if( field.type().msgName() == known_type.msgName()  )
          {
            field.changeType( known_type );
            break;
          }
        }
      }
    }
  }

  for(const auto& msg: parsed_msgs)
  {
    library.insert( {msg->type().baseName(), msg} );
  }
}

} // end namespace


