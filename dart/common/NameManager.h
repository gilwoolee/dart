/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>,
 *            Michael Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_COMMON_NAMEMANAGER_H_
#define DART_COMMON_NAMEMANAGER_H_

#include <map>
#include <sstream>
#include "dart/common/Console.h"

namespace dart {
namespace common {

/// \brief class NameManager
///
/// Typical usage:
/// \code{.cpp}
/// using namespace dart;
///
/// NameManager<BodyNode*> nameMgr;
///
/// BodyNode* bodyNode = new BodyNode();
/// std::string name = "Link";
///
/// if (!nameMgr.hasName(name)
///   nameMgr.addName(name, bodyNode);  // "Link"
/// else
///   name = nameMgr.issueNewNameAndAdd(name, bodyNode);  // "Link1"
///
/// bodyNode->setName(name);
/// \endcode
template <typename T>
class NameManager
{
public:
  /// Constructor
  NameManager(const std::string& _defaultName = "default")
    : mDefaultName(_defaultName),
      mNameBeforeNumber(true),
      mPrefix(""), mInfix("("), mAffix(")") {}

  /// Destructor
  virtual ~NameManager() {}

  /// Set a new pattern for name generation.
  ///
  /// Use %s to indicate the base name and use %d to indicate where the number
  /// belongs. The pattern must contain both a %s and a %d.
  ///
  /// Examples:
  /// "%s(%d)" : name -> name(1) -> name(2)
  /// "%d-%s" : name -> 1-name -> 2-name
  ///
  /// returns false if the pattern was invalid (i.e. did not contain b
  /// oth %s and %d)
  bool setPattern(const std::string& _newPattern)
  {
    size_t name_start = _newPattern.find("%s");
    size_t number_start = _newPattern.find("%d");

    if(name_start == std::string::npos || number_start == std::string::npos)
      return false;

    if(name_start < number_start)
      mNameBeforeNumber = true;
    else
      mNameBeforeNumber = false;

    size_t prefix_end = std::min(name_start, number_start);
    size_t infix_end = std::max(name_start, number_start);

    mPrefix = _newPattern.substr(0, prefix_end);
    mInfix = _newPattern.substr(prefix_end+2, infix_end-prefix_end-2);
    mAffix = _newPattern.substr(infix_end+2);

    return true;
  }

  /// Issue new unique combined name of given base name and number suffix
  std::string issueNewName(const std::string& _name) const
  {
    if(!hasName(_name))
      return _name;

    int count = 1;
    std::string newName;
    do
    {
      std::stringstream ss;
      if(mNameBeforeNumber)
          ss << mPrefix << _name << mInfix << count++ << mAffix;
      else
          ss << mPrefix << count++ << mInfix << _name << mAffix;
      newName = ss.str();
    } while (hasName(newName));

    dtwarn << "The name '" << _name << "' is a duplicate, "
           << "so it has been renamed to '" << newName << "'\n";

    return newName;
  }

  /// Call issueNewName() and add the result to the map
  std::string issueNewNameAndAdd(const std::string& _name, T* _obj)
  {
    const std::string& checkEmpty = _name.empty()? mDefaultName : _name;
    const std::string& newName = issueNewName(checkEmpty);
    addName(newName, _obj);

    return newName;
  }

  /// Add an object to the map
  bool addName(const std::string& _name, T* _obj)
  {
    if (_name.empty())
    {
      dtwarn << "Empty name is not allowed.\n";
      return false;
    }

    if (hasName(_name))
    {
      dtwarn << "Name [" << _name << "] already exist.\n";
      return false;
    }

    mMap.insert(std::pair<std::string, T*>(_name, _obj));

    return true;
  }

  /// Remove an object from the map based on its name
  bool removeName(const std::string& _name)
  {
    typename std::map<std::string, T*>::iterator it = mMap.find(_name);

    if (it == mMap.end())
      return false;

    mMap.erase(it);

    return true;
  }

  /// Clear all the objects
  void clear()
  {
    mMap.clear();
  }

  /// Return true if the name is contained
  bool hasName(const std::string& _name) const
  {
    return (mMap.find(_name) != mMap.end());
  }

  /// Get the number of the objects currently stored by the NameManager
  size_t getCount() const
  {
    return mMap.size();
  }

  /// Get object by given name
  /// \param[in] _name
  ///   Name of the requested object
  /// \return
  ///   The object if it exists, or NULL if it does not exist
  T* getObject(const std::string& _name) const
  {
    typename std::map<std::string, T*>::const_iterator result =
        mMap.find(_name);

    if (result != mMap.end())
      return result->second;
    else
      return NULL;
  }

  /// Set the name that will be provided to objects passed in with an empty
  /// string for a name
  void setDefaultName(const std::string& _defaultName)
  {
    mDefaultName = _defaultName;
  }

  /// Get the name that will be provided to objects passed in with an empty
  /// string for a name
  const std::string& getDefaultName() const
  {
    return mDefaultName;
  }

protected:
  /// Map of objects that have been added to the NameManager
  std::map<std::string, T*> mMap;

  /// String which will be used as a name for any object which is passed in with
  /// an empty string name
  std::string mDefaultName;

  /// Internal variable used to arrange the text when resolving duplicate names
  bool mNameBeforeNumber;

  /// The chunk of text that gets prepended to a duplicate name
  std::string mPrefix;

  /// The chunk of text that comes between a duplicate name and its duplication
  /// number
  std::string mInfix;

  /// The chunk of text that gets appended to a duplicate name
  std::string mAffix;
};

} // namespace common
} // namespace dart

#endif // DART_COMMON_NAMEMANAGER_H_
