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

#include "serializable.h"

namespace boss {

class IdContext;

class Identifiable: public Serializable {

public:
  Identifiable(int id=-1, IdContext* context=0);
  virtual ~Identifiable();
  bool setId(int id, IdContext* context=0);
  bool setContext(IdContext* context);
  IdContext* getContext() {return _context;}
  void ensureValidId(IdContext* context);
  int getId();
  
  virtual void serialize(ObjectData& data, IdContext& context);
  virtual void deserialize(ObjectData& data, IdContext& context);
  
  friend class IdContext;
protected:
  int _id;
  
private:
  //An Identifiable object is unique by definition and cannot be copied
  Identifiable& operator=(const Identifiable& other);
  Identifiable(const Identifiable& other);
  IdContext* _context;
};

}
