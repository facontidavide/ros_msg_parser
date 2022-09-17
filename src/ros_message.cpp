/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright 2016-2017 Davide Faconti
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
* *******************************************************************/


#include <algorithm>

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
  // From http://wiki.ros.org/msg#Fields
  //   When embedding other Message descriptions, the type name may be relative
  //   (e.g. "Point32") if it is in the same package; otherwise it must be the
  //   full Message type (e.g. "std_msgs/String"). The only exception to this
  //   rule is Header.
  for (ROSField& field: _fields)
  {
    // if package name is missing, try to find pkgName/msgName in the list of all_types
    if( field.type().pkgName().size() != 0 ) continue;
    const auto found = std::find_if(
        all_types.begin(), all_types.end(),
        [&](const ROSType* known_type) {
          return ( (this->type().pkgName().compare( known_type->pkgName() ) == 0) &&
                   (field.type().msgName().compare( known_type->msgName() ) == 0) );
        });
    if (found != all_types.end())
    {
      field._type.setPkgName( (*found)->pkgName() );
    }
    else if ( field.type().msgName() == "Header" )
    {
      field._type.setPkgName( "std_msgs" );
    }
  }
}

} // end namespace
