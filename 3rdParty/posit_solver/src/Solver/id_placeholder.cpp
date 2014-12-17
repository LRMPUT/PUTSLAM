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


#include "../../include/Solver/id_placeholder.h"

using namespace boss;
using namespace std;

void IdPlaceholder::resolve(Identifiable* instance) {
  for (vector<AbstractPlaceHolderAssigner*>::iterator it=_assigners.begin();it!=_assigners.end();it++) {
    (*it)->assign(instance);
  }
}

IdPlaceholder::~IdPlaceholder() {
  for (vector<AbstractPlaceHolderAssigner*>::iterator it=_assigners.begin();it!=_assigners.end();it++) {
    delete *it;
  }
}


