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

#include <core1serialization_binary/binarywriter.h>

/** Questo � un esempio basilare della nuova serializzazione. */

using namespace RDK2;

class B: public Serializable {
	char dieci[10];
	
	void write(Writer*) throw (WritingException);
	void read(Reader*)  throw (ReadingException);
};

class A: public Serializable {
	B becero;
	int cavallo;

	void write(Writer*) throw (WritingException);
	void read(Reader*r)  throw (ReadingException);
};

	void B::write(Writer*) {
		r->startWriting("b");
			r->writeBuffer("dieci", 10, buffer);
		r->doneWriting();
	}
	
	void B::read(Reader*r)  throw (FormatE) {
		r->startReading("b");
			r->loadBuffer("dieci", 10, buffer);
		r->doneReading();
	}
	
	void A::write(Reader*r)  throw (FormatE) {
		r->startWriting("a");
			r->writeObject("becero", &becero);
			r->writeInt("cavallo");
		r->doneWriting();
	}

	void A::read(Reader*r)  throw (FormatE) {
		r->startReading("a");
			r->loadObject("becero", &becero);
			cavallo = r->loadInt("cavallo");
			if(cavallo<=0) throw FormatE("Dove vado senza un cavallo?");
		r->doneReading();
	}

	
void main() {
	// I nostri dati
	A a; a.cavallo = 12;
	
	// Scrittura come xml
	XMLWriter xmlWriter; 
	string xml = xmlWriter.serialize(&a);
	
	// Scrittura come stringa binaria
	BinaryWriter binaryWriter;
	string binary = binaryWriter.serialize(&a);
	
	// Lettura da xml
	XMLReader xmlReader;
	A a2;
	xmlReader.deserialize(xml, &a2);	
	// Lettura (anonima) da xml
	Readable * readable = xmlReader.deserialize(xml);
	
	// Lettura binaria
	BinaryReader binaryReader;
	A a3;
	binaryReader.deserialize(binary, &a3);
	// Lettura (anonima) binaria: ovviamente non possibile 
}
