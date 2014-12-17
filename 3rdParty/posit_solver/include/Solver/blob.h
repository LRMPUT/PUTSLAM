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
#include <iostream>
#include <fstream>
#include <memory>

#include "identifiable.h"

namespace boss {

class BaseBLOBReference;

template <class T> class BLOBReference;

class BLOB {
  template <class T>
  friend class BLOBReference;
public:
  BLOB(): _ref(0) {}
  virtual bool read(std::istream& is)=0;
  virtual void write(std::ostream& os)=0;
  virtual ~BLOB();
  virtual const std::string& extension();
protected:
  BaseBLOBReference* _ref;
};

class BaseBLOBReference: public Identifiable {
public:
  BaseBLOBReference(BLOB* instance, int id, IdContext* context):
    Identifiable(id, context), _instance(instance) {
    _nameAttribute="unknown";
  }

  virtual void serialize(ObjectData& data, IdContext& context);
  virtual void deserialize(ObjectData& data, IdContext& context);
  virtual BLOB* get()=0;
  virtual void set(BLOB*);
  void dataDestroyed();
  const std::string& extension();
  inline const std::string& nameAttribute() {return _nameAttribute;}
  inline void setNameAttribute(const std::string& nameAttribute_) {_nameAttribute = nameAttribute_;}

  void setFileName(const std::string& fname) {
    _fileName=fname;
  }
  
  const std::string& getFileName() {
    return _fileName;
  }
  
  virtual ~BaseBLOBReference();

  inline BLOB* instance() {return _instance;}
protected:
  bool load(BLOB& instance);
  
  std::string _fileName;
  std::string _nameAttribute;
  BLOB* _instance;
};

template<typename T>
class BLOBReference: public BaseBLOBReference {
public:    
  BLOBReference(T* instance=0, int id = -1, IdContext* context = 0):
    BaseBLOBReference(instance, id, context) {}
  
  virtual T* get();
};

template <typename T>
T* BLOBReference<T>::get() {
  if (!_instance && getContext()) {
    std::auto_ptr<T> blob(new T());
    if (load(*blob)) {
      _instance=blob.get();
      blob->_ref = this;
      blob.release();
    } 
 }
  // std::cerr << "get: " << className() 
  // 	    << " instance: "<< _instance 
  // 	    << " ptr: " << this << " context: " << getContext() << std::endl;
  return (T*) _instance;
}

}

#define BOSS_REGISTER_BLOB(class_name) \
  static boss::AutoRegisterer<boss::BLOBReference<class_name > > _reg_##class_name(#class_name,typeid(BLOBReference<class_name >).name());
