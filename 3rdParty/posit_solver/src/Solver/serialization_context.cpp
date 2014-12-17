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
#include "../../include/Solver/serialization_context.h"
#include "../../include/Solver/serializer.h"

using namespace boss;
using namespace std;
using namespace boost::filesystem;


static const string DEFAULT_DATA_FILE="data.log";
static const string DEFAULT_BLOB_FILE="binary/<classname>.<nameAttribute>.<id>.<ext>";
static const string CURRENT_DIR=".";


static string toString(int i, size_t width, char padding) {
  stringstream ss;
  ss.width(width);
  ss.fill(padding);
  ss <<  i;
  return ss.str();
}

void SerializationContext::loadCurrentTime() {
  time_t currentTime=time(0);
  struct tm* timeStruct=localtime(&currentTime);
  _envMap["yyyy"]=toString(timeStruct->tm_year+1900,4,'0');
  _envMap["yy"]=toString((timeStruct->tm_year+1900)%100,2,'0');
  _envMap["mm"]=toString(timeStruct->tm_mon+1,2,'0');
  _envMap["dd"]=toString(timeStruct->tm_mday,2,'0');
  _envMap["hh"]=toString(timeStruct->tm_hour,2,'0');
  _envMap["mi"]=toString(timeStruct->tm_min,2,'0');
  _envMap["ss"]=toString(timeStruct->tm_sec,2,'0');
}

void SerializationContext::replaceEnvTags(string& str) {
  string::iterator beginTag=str.end();
  for (string::iterator it=str.begin();it!=str.end();it++) {
    if (*it=='<') {
      beginTag=it;
      continue;
    }
    if (*it=='>'&&beginTag<it) {
      string replacement=_envMap[str.substr(beginTag+1-str.begin(),it-beginTag-1)];
      size_t newpos=beginTag-str.begin()+replacement.length()-1;
      str.replace(beginTag,it+1,replacement);
      it=str.begin()+newpos;
    }
  }
}


static void adjustBinaryPath(string& fname, map<string,string>& envMap) {
  //Check if it's an absolute path
  if (fname[0]!='/') {
    fname.insert(0,"/");
    fname.insert(0,envMap["datadir"]);
  }
}

SerializationContext::SerializationContext() {
  _inputStream = 0;
  _outputStream = 0;
  setOutputFilePath(DEFAULT_DATA_FILE);
  setBinaryPath(DEFAULT_BLOB_FILE);
}

SerializationContext::~SerializationContext() {
  destroyInputStream();
  destroyOutputStream();
}


string SerializationContext::createBinaryFilePath(BaseBLOBReference& instance) {
  _envMap["classname"]=instance.className();
  _envMap["nameAttribute"]=instance.nameAttribute();
  _envMap["id"]=toString(instance.getId(),7,'0');
  _envMap["ext"]=instance.extension();
  string str=_blobFileName;
  replaceEnvTags(str);
  //cerr << "Binary file path: " << str << endl;
  return str;
}


ostream* SerializationContext::getBinaryOutputStream(const string& fname) {
  if (fname.empty()) {
    return 0;
  }
  string str=fname;
  adjustBinaryPath(str,_envMap);
  if (!path(str).parent_path().empty()) {
    create_directories(path(str).parent_path());
  }
  return new ofstream(str.c_str());
}

istream* SerializationContext::getBinaryInputStream(const string& fname) {
  if (fname.empty()) {
    return 0;
  }
  string str=fname;
  adjustBinaryPath(str,_envMap);
  return new ifstream(str.c_str());
}

void SerializationContext::makeInputStream() {
  if (!_inputStream) {
    if (_inputDataFileName == "stdin"){
      _inputStream=&cin;
    } else
      _inputStream=new ifstream(_inputDataFileName.c_str());
  }
}

void  SerializationContext::makeOutputStream(){
  if (!_outputStream) {
    string str=_outputDataFileName;
    if (str=="stdout"){
      _outputStream = &cout;
    } else
    if (str=="stderr"){
      _outputStream = &cerr;
    } else {
      replaceEnvTags(str);
      if (!path(str).parent_path().empty()) { 
        create_directories(path(str).parent_path());
      }
      _outputStream=new ofstream(str.c_str());
    }
  }
}

void SerializationContext::setInputStream(std::istream* is, const std::string& filename){
  destroyInputStream();
  if (is == &cin){
    _inputDataFileName = "stdin";
  } else if (_inputDataFileName!="")
    _inputDataFileName = filename;
  _inputStream = is;
}

void SerializationContext::setOutputStream(std::ostream* os, const std::string& filename){
  destroyOutputStream();
  if (os == &cout) {
    _outputDataFileName = "stdout";
  } else if (os == &cerr) {
    _outputDataFileName = "stderr";
  } else if (_outputDataFileName!="") {
    _outputDataFileName = filename;
  }
  _outputStream = os;
}

void SerializationContext::destroyInputStream() {
  if (_inputStream && _inputStream!=&cin) {
    delete _inputStream;
  }
  _inputStream=0;

}
 
void SerializationContext::destroyOutputStream() {
 if (_outputStream && _outputStream!=&cerr && _outputStream!=&cout) {
    delete _outputStream;
  }
 _outputStream = 0;
}

void SerializationContext::setInputFilePath(const string& fpath) {
  if (fpath.length()==0) {
    return;
  }
  _inputDataFileName=fpath;
  destroyInputStream();
}

void SerializationContext::setOutputFilePath(const string& fpath) {
  if (fpath.length()==0) {
    return;
  }
  _outputDataFileName=fpath;
  loadCurrentTime();
  //Extract directory
  size_t pos=_outputDataFileName.rfind('/');
  if (pos==string::npos) {
    _envMap["datadir"]=CURRENT_DIR;
  } else {
    _envMap["datadir"]=_outputDataFileName.substr(0,pos);
  }
  destroyOutputStream();
}

void SerializationContext::setBinaryPath(const string& fpath) {
  if (fpath.length()==0) {
    return;
  }
  _blobFileName=fpath;
}
