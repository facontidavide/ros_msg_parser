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

#include <boost/algorithm/string.hpp>
#include <boost/utility/string_ref.hpp>
#include <boost/utility/string_ref.hpp>
#include "ros_msg_parser/ros_message.hpp"
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>

namespace RosMsgParser{

ROSMessage::ROSMessage(const std::string &msg_def)
{
  std::istringstream messageDescriptor(msg_def);
  boost::match_results<std::string::const_iterator> what;

  for (std::string line; std::getline(messageDescriptor, line, '\n') ; )
  {
    std::string::const_iterator begin = line.begin(), end = line.end();

    // Skip empty line or one that is a comment
    if (boost::regex_search( begin, end, what,
                             boost::regex("(^\\s*$|^\\s*#)")))
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

void ROSMessage::updateMissingPkgNames(const std::vector<const ROSType*> &all_types)
{
  for (ROSField& field: _fields)
  {
    // if package name is missing, try to find msgName in the list of known_type
    if( field.type().pkgName().size() == 0 )
    {
      for (const ROSType* known_type: all_types)
      {
        if( field.type().msgName().compare( known_type->msgName() ) == 0  )
        {
          field._type.setPkgName( known_type->pkgName() );
          break;
        }
      }
    }
  }
}

} // end namespace


