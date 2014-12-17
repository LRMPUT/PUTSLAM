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

//#include <fstream>
#include <memory>

#include "../../include/Solver/blob.h"
#include "../../include/Solver/object_data.h"
#include "../../include/Solver/serialization_context.h"
#include "../../include/Solver/id_context.h"

using namespace boss;
using namespace std;

BLOB::~BLOB() {
  if (_ref) {
    _ref->dataDestroyed();
  }
}

static string DEFAULT_EXTENSION("dat");

const string& BLOB::extension() {
  return DEFAULT_EXTENSION;
}

const string& BaseBLOBReference::extension() {
  if (_instance) {
    return _instance->extension();
  }
  return DEFAULT_EXTENSION;
}

void BaseBLOBReference::dataDestroyed() {
  _instance=0;
}

BaseBLOBReference::~BaseBLOBReference() {
  if (_instance) {
    delete _instance;
  }
}

void BaseBLOBReference::serialize(ObjectData& data, IdContext& context) {
  Identifiable::serialize(data,context);
  if (_instance) {
    //Check if binary file serialization is supported
    SerializationContext* fileContext=context.serializationContext();
    if (fileContext) {
      _fileName=fileContext->createBinaryFilePath(*this);
      auto_ptr<ostream> os(fileContext->getBinaryOutputStream(_fileName));
      if (os.get()) {
        _instance->write(*os);
      }
    } 
  }
  data << field("pathName",_fileName);
}

void BaseBLOBReference::deserialize(ObjectData& data, IdContext& context) {
  Identifiable::deserialize(data, context);
  data >> field("pathName",_fileName);
  _instance=0;
}

bool BaseBLOBReference::load(BLOB& instance) {
  //Check if binary file serialization is supported
  SerializationContext* fileContext=getContext()->serializationContext();
  if (fileContext) {
    auto_ptr<istream> is(fileContext->getBinaryInputStream(_fileName));
    if (is.get()) {
      return instance.read(*is);
    }
  } 
  return false;
}

void BaseBLOBReference::set(BLOB* instance) {
  if (_instance) {
    delete _instance;
  }
  _instance=instance;
}
