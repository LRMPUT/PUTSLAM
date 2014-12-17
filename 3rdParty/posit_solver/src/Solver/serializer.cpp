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

#include <ctime>
#include <sstream>
#include <boost/filesystem.hpp>

#include "../../include/Solver/serializer.h"
#include "../../include/Solver/json_message_writer.h"
#include "../../include/Solver/json_object_writer.h"
#include "../../include/Solver/object_data.h"

using namespace std;
using namespace boss;
using namespace boost::filesystem;


 
static ValueData* processDataForWrite(ValueData* vdata, IdContext& context) {
  switch (vdata->type()) {
    case OBJECT: {
      ObjectData* o=static_cast<ObjectData*>(vdata);
      const vector<string>& fields=o->fields();
      for (vector<string>::const_iterator f_it=fields.begin();f_it!=fields.end();f_it++) {
        ValueData* fdata=o->getField(*f_it);
        ValueData* replaceData=processDataForWrite(fdata, context);
        if (replaceData) {
          o->setField(*f_it,replaceData);
        }
      }
      break;
    }
    case ARRAY: {
      ArrayData* v_array=static_cast<ArrayData*>(vdata);
      for (vector<ValueData*>::const_iterator v_it=v_array->begin();v_it!=v_array->end();v_it++) {
        ValueData* replaceData=processDataForWrite(*v_it, context);
        if (replaceData) {
          v_array->set(v_it-v_array->begin(),replaceData);
        }
      }
      break;
    }
    case POINTER: {
      Identifiable* identifiable=vdata->getPointer();
      if (identifiable) {
	identifiable->ensureValidId(&context);
      }
      ObjectData* pointerObject=new ObjectData();
      pointerObject->setInt("#pointer",identifiable?identifiable->getId():-1);
      return pointerObject;
    }
    default:
      //Nothing to do
      break;
  }
  return 0;
}

Serializer::Serializer(SerializationContext *sc){
  _writer=new JSONMessageWriter();
  _objectWriter=new JSONObjectWriter();
  _serializationContext = sc;
}

void Serializer::setFilePath(const string& fpath) {
  if (! _serializationContext)
    _serializationContext = new SerializationContext;
  cerr << __PRETTY_FUNCTION__ << ": obsolete. Use serializationContext::serOutputFilePath instead" << endl;
  _serializationContext->setOutputFilePath(fpath);
}

void Serializer::setBinaryPath(const string& fpath) {
  if (! _serializationContext)
    _serializationContext = new SerializationContext;
  cerr << __PRETTY_FUNCTION__ << ": obsolete. Use serializationContext::setBinaryPath instead" << endl;
  _serializationContext->setBinaryPath(fpath);
}

bool Serializer::setFormat(const string& format) {
  //TODO Implementation
  return format=="JSON";
}

bool Serializer::write(const string& source, Serializable& instance) {
  return write(Message::getCurrentTime(), source, instance);
}

bool Serializer::write(Message& message) {
  return write(message.getTimestamp(), message.getSource(), *message.getInstance());
}

bool Serializer::write(double timestamp, const string& source, Serializable& instance) {
  ObjectData* data=new ObjectData();
  instance.serialize(*data,*this);
  
  processDataForWrite(data,*this);

  MessageData msgData(timestamp, instance.className(), source, data);
  _serializationContext->makeOutputStream();
  if (*_serializationContext->outputStream()) {
    _writer->writeMessage(*_serializationContext->outputStream(),msgData);
    //TODO Change writer to get status flag
    return true;
  }
  return false;
}

bool Serializer::writeObject(Serializable& instance) {
  ObjectData* data=new ObjectData();
  instance.serialize(*data,*this);

  processDataForWrite(data,*this);
  _serializationContext->makeOutputStream();
  if (*_serializationContext->outputStream()) {
    _objectWriter->writeObject(*_serializationContext->outputStream(),instance.className(),*data);
    //TODO Change writer to get status flag
    return true;
  }
  return false;

}

bool Serializer::writeObject(std::string& output, Serializable& instance) {
  ObjectData* data=new ObjectData();
  instance.serialize(*data,*this);
  ostringstream os;
  processDataForWrite(data,*this);
  _objectWriter->writeObject(os,instance.className(),*data);
  output = os.str();
  return true;
}

Serializer::~Serializer() {
  delete _writer;
  delete _objectWriter;
}

