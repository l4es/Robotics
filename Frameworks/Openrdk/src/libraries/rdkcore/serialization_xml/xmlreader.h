/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi (<first_name>.<last_name>@dis.uniroma1.it)
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

#ifndef RDK2_SERIALIZATION_XML_READER
#define RDK2_SERIALIZATION_XML_READER

#include <vector>
#include <iostream>
#include <sstream>
#include <libxml/xmlreader.h>
#include <rdkcore/serialization/read.h>

namespace RDK2 { namespace Serialization { namespace Xml {

/** 
* Here it is the algorithm for dealing with names:
* If (name is specified AND element name != name)
*    throw exception.
*/
class XmlReader: public Reader {
public:
	XmlReader() : currentElementIsEmpty(false), checkBeforeGoingToNext(true), alreadyRead(false) { }
	virtual ~XmlReader() { }

	Readable * deserialize(cstr) throw (ReadingException);
	void deserialize(cstr, Readable *) throw (ReadingException);
	
///
/// Reader interface
///
public:
	/** If next packet is not of specified class name, throws exception. */
	unsigned char startReading(cstr className) throw (ReadingException);
		
		uint8_t read_u8(cstr name) throw (ReadingException);
		int8_t read_i8(cstr name) throw (ReadingException);
		int16_t read_i16(cstr name) throw (ReadingException);
		int32_t read_i32(cstr name) throw (ReadingException);
		float32_t read_f32(cstr name) throw (ReadingException);
		float64_t read_f64(cstr name) throw (ReadingException);
		
		void read_u8(uint8_t**, size_t* n, cstr name) throw (ReadingException);
		void read_i8(int8_t**, size_t* n, cstr name) throw (ReadingException);
		void read_i16(int16_t**, size_t* n, cstr name) throw (ReadingException);
		void read_i32(int32_t**, size_t* n, cstr name) throw (ReadingException);
		void read_f32(float32_t**, size_t* n, cstr name) throw (ReadingException);
		void read_f64(float64_t**, size_t* n, cstr name) throw (ReadingException);
		
		void readBytes(void**, size_t* n, cstr name) throw (ReadingException);
		std::string readString(cstr name) throw (ReadingException);

		void readObject(Readable* r, cstr name = "") throw (ReadingException);
		Readable* readObject(cstr name = "") throw (ReadingException);
		
	void doneReading() throw (ReadingException);

	void error(cstr error) throw (ReadingException) { throwIt(error); };

///
/// Private stuff
///
private:
	xmlTextReader* reader;
	bool currentElementIsEmpty, checkBeforeGoingToNext;
	std::string curObjectName;
	int innerObjects;
	bool alreadyRead;

	bool goToElementOrEndElement();
	void iWantAnElement(cstr elementName, bool dontCareIfEmpty = true) throw (ReadingException);
	void iWantANamedElement(cstr elementName, cstr name, bool dontCareIfEmpty = true) throw (ReadingException);
	void iWantANamedElementArrayOfType(cstr name, cstr type) throw (ReadingException);
	std::string getAttributeValue(cstr attributeName) throw (ReadingException);
	void iWantAnEndElement(cstr elementName) throw (ReadingException);

	// FIXME AC  Non funziona con gli array di lunghezza 0 (size=0)
	// vedi anche readArrayOfBytes
	// FIXME AC  Usare le funzioni in geometry/utils.h per parsare bene "inf" e "nan"
	template<typename T>
	void readArray(T** a, size_t *n)  throw (ReadingException)
	{
		size_t sz = atoi(getAttributeValue("size").c_str());
		xmlTextReaderRead(reader);
		if (sz == 0) throwIt("No 'size' attribute in array element");
		*a = new T[sz];
		const char* xs = (const char*) xmlTextReaderConstValue(reader);
		stringstream ss(string((xs ? xs : "")));
		for (size_t i = 0; i < sz; i++) {
			bool b = (ss >> (*a)[i]);
			if (!b) throwIt(string() + "I expected more array values (string was: '" + ss.str() + "')");
		}
		*n = sz;
	}
	void readArrayOfBytes(uint8_t** a, size_t *n)  throw (ReadingException);

	void throwIt(cstr error) throw (ReadingException);
};

}}} // namespace

#endif
