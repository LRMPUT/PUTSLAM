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

#include <limits>
#include <stdexcept>

#include "../../include/Solver/json_message_writer.h"
#include "../../include/Solver/object_data.h"

using namespace std;
using namespace boss;

static string escapeJSONString(const string& str) {
  string escaped;
  for (string::const_iterator c_it=str.begin();c_it!=str.end();c_it++) {
    char c=*c_it;
    switch (c) {
    case '\\':
    case '"':
      escaped.push_back('\\');
      escaped.push_back(c);
      break;
    case '\n':
      escaped.push_back('\\');
      escaped.push_back('n');
      break;
    case '\b':
      escaped.push_back('\\');
      escaped.push_back('b');
      break;
    case '\f':
      escaped.push_back('\\');
      escaped.push_back('f');
      break;
    case '\r':
      escaped.push_back('\\');
      escaped.push_back('r');
      break;
    case '\t':
      escaped.push_back('\\');
      escaped.push_back('t');
      break;
    default:
      escaped.push_back(c);
    }
  }
  return escaped;
}

static void writeJSONData(ostream& os, ValueData* val) {
  switch (val->type()) {
  case BOOL:
    os << val->getBool();
    break;
  case NUMBER:
    os.precision(static_cast<NumberData*>(val)->precision());
    os << val->getDouble();
    break;
  case STRING:
    os << '"' << escapeJSONString(val->getString()) << '"';
    break;
  case ARRAY: {
    os << "[ ";
    ArrayData* v_array=static_cast<ArrayData*>(val);
    for (vector<ValueData*>::const_iterator v_it=v_array->begin();v_it!=v_array->end();v_it++) {
      if (v_it!=v_array->begin()) {
        os << ", ";
      }
      writeJSONData(os,*v_it);
    }
    os << " ]";
    break;
  }
  case OBJECT: {
    os << "{ ";
    ObjectData* o=static_cast<ObjectData*>(val);
    const vector<string>& fields=o->fields();
    for (vector<string>::const_iterator f_it=fields.begin();f_it!=fields.end();f_it++) {
      if (f_it!=fields.begin()) {
        os << ", ";
      }
      ValueData* fdata=o->getField(*f_it);
      os << '"' << escapeJSONString(*f_it) << "\" : ";
      writeJSONData(os,fdata);
    }
    os << " }";
    break;
  }
  default:
    throw logic_error("unexpected value type: "+val->typeName());
  }
}

void JSONMessageWriter::writeMessage(ostream& os, MessageData& message) {
  int precision=os.precision();
  //Message base data
  os.precision(numeric_limits<double>::digits10);
  os << message.getTimestamp() << " \"" << escapeJSONString(message.getType()) << "\" \"" << escapeJSONString(message.getSource()) << "\" ";
  ObjectData* data=message.getData();
  writeJSONData(os,data);
  os << endl;
  os.precision(precision);
}
