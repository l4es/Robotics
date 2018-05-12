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

#include <iostream>
#include <cstring>
#include <rdkcore/serialization_binary/binarywriter.h>
#include <rdkcore/serialization_binary/binaryreader.h>

/** Questo � un esempio basilare della nuova serializzazione. */

using namespace RDK2::Serialization;
using namespace RDK2::Serialization::Binary;

struct B: public Writable, public Readable {
	char dieci[10];
	
	B() {
		strcpy(dieci, "123451234");	
	}
	void write(Writer*) const throw (WritingException);
	void  read(Reader*) throw (ReadingException);
};

struct A: public Writable, public Readable {
	B becero;
	int cavallo;

	void write(Writer*) const  throw (WritingException) ;
	void  read(Reader*) throw (ReadingException);
};

	void B::write(Writer*r) const throw(WritingException) {
		r->startWriting("b");
			r->writeBytes(dieci, 10, "dieci");
		r->doneWriting();
	}
	
	void B::read(Reader*r)  throw (ReadingException) {
		r->startReading("b");
			int8_t * buffer; size_t n;
			r->read_i8( &buffer, &n, "dieci");
			if(n!=10)
				r->error("I expected a 10 byte buffer");
			memcpy(dieci, buffer, 10);
			delete[] buffer;
		r->doneReading();
	}
	
	void A::write(Writer*r)  const throw (WritingException) {
		r->startWriting("a");
		//	r->writeObject(true, &becero, "becero");
		//	r->write_i32(cavallo,"cavallo");
		r->doneWriting();
	}

	void A::read(Reader*r)  throw (ReadingException) {
		r->startReading("a");
		//	r->readObject( &becero, "becero");
		//	cavallo = r->read_i32("cavallo");
		//	if(cavallo<=0) 
		//		r->error("Dove vado senza un cavallo?");
		r->doneReading();
	}
	
int main() {
	try {
		// I nostri dati
		A a; a.cavallo = 12;
		
		// Scrittura come stringa binaria
		BinaryWriter binaryWriter(true);
		string binary = binaryWriter.serialize(true,&a);
		
		std::cout << binary;

		BinaryReader binaryReader;
		
		A a2;
		binaryReader.deserialize(binary, &a2);
	} catch(SerializationException e) {
		std::cerr << e.what();	
	}
}
