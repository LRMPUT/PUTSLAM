/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include <string>

namespace boss {

class Serializable;

class Message {
public:
  Message() {};
  
  /*!
   * Create a new message with the specified data, note that this class does not take ownership of the
   * passed Serializable instance.
   * \param timestamp the message timestamp
   * \param source a string identifying the source that generated this message
   * \param instance the Serializable object to be written
   */
  Message(double timestamp, const std::string& source, Serializable* instance):
    _timestamp(timestamp),
    _source(source),
    _instance(instance) {}
  
  double getTimestamp() {
    return _timestamp;
  }
  
  void setTimestamp(double ts) {
    _timestamp=ts;
  }
  
  const std::string& getSource() {
    return _source;
  }

  void setSource(const std::string& source) {
    _source=source;
  }

  Serializable* getInstance() {
    return _instance;
  }
  
  void setInstance(Serializable* instance) {
    _instance=instance;
  }
  
  virtual ~Message() {};
  
  static double getCurrentTime();
  
protected:
  double _timestamp;
  std::string _source;
  Serializable* _instance;
};

}
