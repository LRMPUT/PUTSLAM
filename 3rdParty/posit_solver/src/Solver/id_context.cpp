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


#include "../../include/Solver/id_context.h"

#include "../../include/Solver/identifiable.h"
#include "../../include/Solver/id_placeholder.h"
#include <stdexcept>

using namespace boss;
using namespace std;
using boost::mutex;
using boost::lock_guard;

IdContext::IdContext(): _lastGeneratedID(0) {
  _serializationContext = 0;
}

bool IdContext::add(Identifiable* obj) {
  lock_guard<mutex> lock(_instances_lock);
  return addImpl(obj);
}

bool IdContext::addImpl(Identifiable* obj) {
  //cerr << "add obj:" << obj << " context:" << this;
  Identifiable*& instance=_instances[obj->getId()];
  if (instance) {
    IdPlaceholder* placeHolder=dynamic_cast<IdPlaceholder*>(instance);
    if (placeHolder) {
      placeHolder->resolve(obj);
      placeHolder->_context=0;
      delete placeHolder;
    } else {
      return true;
      //return false;
    } 
  }
  instance=obj;
  //cerr << " true" << endl;
  return true;
}

bool IdContext::remove(Identifiable* obj) {
//  lock_guard<mutex> lock(_instances_lock);
  return _instances.erase(obj->getId())>0;
}

bool IdContext::update(Identifiable* obj, int oldId) {
  if (obj->getId()==oldId) {
    return true;
  }
  lock_guard<mutex> lock(_instances_lock);
  if (addImpl(obj)) {
    _instances.erase(oldId);
    return true;
  }
  return false;
}

Identifiable* IdContext::getById(int id) {
  lock_guard<mutex> lock(_instances_lock);
  map<int, Identifiable*>::iterator instance_pair=_instances.find(id);
  if (instance_pair==_instances.end()) {
    return 0;
  }
  return (*instance_pair).second;
}

void IdContext::setStartingId(int id){
  if (id<0)
    throw std::runtime_error("ids cannot be less than 0");
  _lastGeneratedID=id;
}

int IdContext::generateId() {
  lock_guard<mutex> lock(_instances_lock);
  int minId=_lastGeneratedID+1;
  if (!_instances.empty()) {
    int curId=(*_instances.rbegin()).first+1;
    if (curId>minId) {
      minId=curId;
    }
  }
  return _lastGeneratedID=minId;
}

IdPlaceholder* IdContext::createPlaceHolder(int id) {
  return new IdPlaceholder(id,this);
}

IdContext::~IdContext() {
  lock_guard<mutex> lock(_instances_lock);
  for (map<int, Identifiable*>::iterator ipair=_instances.begin();ipair!=_instances.end();ipair++) {
    Identifiable* instance=(*ipair).second;
    if (dynamic_cast<IdPlaceholder*>(instance)) {
      delete instance;
    } else {
      instance->_context=0;
    }
  }
  _instances.clear();
}

