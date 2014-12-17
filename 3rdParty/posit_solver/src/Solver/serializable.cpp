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

#include <stdexcept>
#include <typeinfo>
#include <sys/time.h>

#include "../../include/Solver/serializable.h"
#include "../../include/Solver/object_data.h"

using namespace std;
using namespace boss;

static map<string, Serializable* (*)()>& factoryMap() {
  static map<string, Serializable* (*)()> FACTORY_MAP;
  return FACTORY_MAP;
}

static map<string, string>& typeIdMap() {
  static map<string, string> TYPEID_MAP;
  return TYPEID_MAP;
}

vector<string> boss::getClassNames() {
    vector<string> r;
    map<string, Serializable* (*)()>& fm = factoryMap();
    for (map<string, Serializable*(*)()>::iterator it = fm.begin(); it != fm.end(); ++it) {
        r.push_back(it->first);
    }
    return r;
}

void Serializable::registerFactory(const string& className, const string& typeIdName, Serializable* (*func)()) {
  cerr << "Registered class " << className << endl;
  factoryMap()[className]=func;
  typeIdMap()[typeIdName]=className;
}

Serializable* Serializable::createInstance(const string& className) {
  map<string,Serializable* (*)()>::iterator function=factoryMap().find(className);
  if (function==factoryMap().end()) {
    throw logic_error("no factory function mapped for type "+className);
  }
  return (*function).second();
}

const string& Serializable::className() {
  map<string,string>::iterator cname=typeIdMap().find(typeid(*this).name());
  if (cname==typeIdMap().end()) {
    throw logic_error("className() called on unregistered class "+string(typeid(*this).name()));
  }
  return (*cname).second;
}

ObjectData* Serializable::getSerializedData(IdContext& context) {
  ObjectData* o=new ObjectData();
  serialize(*o,context);
  return o;
}

void Serializable::deserializeComplete() {}

double Serializable::getCurrentTime() {
  timeval time;
  gettimeofday(&time, 0);
  return time.tv_sec+(time.tv_usec/1000000.0);
}

