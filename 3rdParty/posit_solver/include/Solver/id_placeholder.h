/*
    Placeholder class for Identifiable objects
    Copyright (C) 2013  Daniele Baldassari <daniele@dikappa.org>

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

#include <vector>

#include "identifiable.h"

namespace boss {

class IdContext;

class AbstractPlaceHolderAssigner {
public:
  virtual void assign(Identifiable* instance)=0;
  virtual ~AbstractPlaceHolderAssigner() {}
};

template<typename T> class PlaceHolderAssigner: public AbstractPlaceHolderAssigner {
public:
  PlaceHolderAssigner(T*& var): _var(&var) {}
  virtual void assign(Identifiable* instance) {
    *_var=dynamic_cast<T*>(instance);
  }
  virtual ~PlaceHolderAssigner() {}

protected:
  T** _var;
};

class IdPlaceholder: virtual public Identifiable {
public:
  template<typename T> void addVariable(T*& var) {
    _assigners.push_back(new PlaceHolderAssigner<T>(var));
  }
  void resolve(Identifiable* instance);
  virtual ~IdPlaceholder();

  friend class IdContext;

protected:
  IdPlaceholder(int id, IdContext* context): Identifiable(id, context) {}
  
  std::vector<AbstractPlaceHolderAssigner*> _assigners;
};

}
