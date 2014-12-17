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

#include "blob.h"

namespace boss {

  class Serializer;

  class SerializationContext {  
  public:
    SerializationContext();
    virtual ~SerializationContext();
    virtual std::string createBinaryFilePath(BaseBLOBReference& instance);
    virtual std::ostream* getBinaryOutputStream(const std::string& fname);
    virtual std::istream* getBinaryInputStream(const std::string& fname);
    virtual void setInputFilePath(const std::string& str);
    virtual void setOutputFilePath(const std::string& str);
    virtual void setBinaryPath(const std::string& str);
    void replaceEnvTags(std::string& str);
    void loadCurrentTime();
    void makeInputStream();
    void makeOutputStream();

    void setInputStream(std::istream* is, const std::string& filename="");
    void setOutputStream(std::ostream* os, const std::string& filename="");

    void destroyInputStream();
    void destroyOutputStream();

    inline std::istream* inputStream() {return _inputStream;}
    inline std::ostream* outputStream() {return _outputStream;}

  protected:
    std::string _inputDataFileName;
    std::string _outputDataFileName;
    std::string _blobFileName;
    std::map<std::string, std::string> _envMap;
    std::istream* _inputStream;
    std::ostream* _outputStream;
  };

}
