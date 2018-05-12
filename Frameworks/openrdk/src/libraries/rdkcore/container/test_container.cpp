/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */



#include <vector>
#include <iostream>
#include <rdkcore/rprimitive/rint.h>
#include <rdkcore/rprimitive/rdouble.h>

#include <rdkcore/container/container.h>

using namespace std;
using namespace RDK2::Containers;
using namespace RDK2::RPrimitive;

typedef RDK2::Containers::Vector<RInt> RIntVector;
typedef RDK2::Containers::Vector<RDouble> RDoubleVector;
typedef RDK2::Containers::Vector<RString> RStringVector;

int main() {
		
	RObjectVector v;
	v.push_back(new RInt(2));
	v.push_back(new RDouble(2.5));
	v.push_back(new RInt(3));
	v.push_back(new RDouble(3.5));
	v.push_back(new RInt(4));
	v.push_back(new RDouble(4.5));
	v.push_back(new RInt(5));

	for(RObjectVector::iterator i=v.begin(); i!=v.end(); i++) {
		cout << (*i)->getStringRepresentation() << endl;
	}

	cout << "Const: " << endl;	
	
	 const RObjectVector& v2 = v; 
	 for(RObjectVector::const_iterator i=v2.begin(); i!=v2.end(); i++) {
		 cout << (*i)->getStringRepresentation() << endl;
	 }

	cout << "Int: " << endl;
	RIntVector vi; vi.copy(v);
	for(RIntVector::const_iterator i=vi.begin(); i!=vi.end(); i++) {
		cout << (*i)->getStringRepresentation() << endl;
	}

	cout << "Double: " << endl;	
	RDoubleVector vd; vd.copy(v);
	for(RDoubleVector::const_iterator i=vd.begin(); i!=vd.end(); i++) {
		cout << (*i)->getStringRepresentation() << endl;
	}

	
}
