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

#include <map>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include "serialization_context.h"
namespace boss {

class Identifiable;
class IdPlaceholder;

class IdContext {
public:
  IdContext();
  Identifiable* getById(int id);
  bool add(Identifiable* obj);
  bool remove(Identifiable* obj);
  bool update(Identifiable* obj, int oldId);
  IdPlaceholder* createPlaceHolder(int id);
  int generateId();
  void setStartingId(int id);
  virtual ~IdContext();
  inline SerializationContext* serializationContext() {return _serializationContext;}
protected:
  virtual bool addImpl(Identifiable* obj);
  
  std::map<int, Identifiable*> _instances;
  boost::mutex _instances_lock;
  int _lastGeneratedID;
  SerializationContext* _serializationContext;
};


}
