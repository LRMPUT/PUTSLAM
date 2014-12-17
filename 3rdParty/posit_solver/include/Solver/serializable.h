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
#include <string>
#include <vector>
#include <iostream>
#include <typeinfo>
#include "trusted_loaders.h"

namespace boss {

class ObjectData;
class IdContext;

std::vector<std::string> getClassNames();

class Serializable {
public:
  /*!
   * Save all your data here, don't forget to call superclass serialize method if not inheriting directly from
   * serializable
   */
  virtual void serialize(ObjectData& data, IdContext& context)=0;
  /*!
   * Restore object status here
   */
  virtual void deserialize(ObjectData& data, IdContext& context)=0;

  /*!
   * Deserializazion callback method, invoked by deserializer when all dangling pointers have been resolved
   */
  virtual void deserializeComplete();

  virtual ~Serializable() {}

  ObjectData* getSerializedData(IdContext& context);
  const std::string& className();
  
  static void registerFactory(const std::string& className, const std::string& typeIdName, Serializable* (*func)());
  
  /*!
   * throws std::logic_error if no factory method was registered for this class namespace
   */
  static Serializable* createInstance(const std::string& className);

  /*!
   * Utility function to get the current timestamp, for time-sensitive data
   */
  static double getCurrentTime();

};

template <class T> class AutoRegisterer {
public:
  AutoRegisterer(const char* className, const char* typeIdName) {
    Serializable::registerFactory(className, typeIdName, &createInstance);
  }
  
  static Serializable* createInstance() {
    return new T();
  }
};

#define BOSS_REGISTER_CLASS(class_name) \
  static boss::AutoRegisterer<class_name > _reg_##class_name(#class_name,typeid(class_name).name());

#define BOSS_REGISTER_CLASS_WITH_NAME(class_name, type_name) \
  static boss::AutoRegisterer<class_name > _reg_##class_name(#type_name,typeid(class_name).name()));

}
