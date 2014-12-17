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

class ObjectData;

class MessageData {
public:
  MessageData();
  
  MessageData(double timestamp, const std::string& type, const std::string& source, ObjectData* data):
    _timestamp(timestamp),
    _type(type),
    _source(source),
    _data(data) {}
  
  double getTimestamp() {
    return _timestamp;
  }
  
  void setTimestamp(double ts) {
    _timestamp=ts;
  }
  
  const std::string& getType() {
    return _type;
  }
  
  void setType(const std::string& type) {
    _type=type;
  }
  
  const std::string& getSource() {
    return _source;
  }

  void setSource(const std::string& source) {
    _source=source;
  }

  ObjectData* getData() {
    return _data;
  }
  
  virtual ~MessageData();
  
protected:
  double _timestamp;
  std::string _type;
  std::string _source;
  ObjectData* _data;
};

}
