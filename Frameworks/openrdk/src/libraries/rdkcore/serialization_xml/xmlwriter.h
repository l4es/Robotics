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

#ifndef H_XML_WRITER
#define H_XML_WRITER

#include <sstream>
#include <libxml/xmlwriter.h>
#include <rdkcore/serialization/write.h>

#include "xmlutils.h"

#define XML_INDENTATION_STRING "  "
#define XML_INTEGER_VALUE_PRINTF_FORMAT "%d"
#define XML_FLOATING_VALUE_PRINTF_FORMAT "%.2f"

namespace RDK2 { namespace Serialization { namespace Xml {

template<typename T>
std::string vtos(T v, const char* format)
{
	char buffer[512];
	sprintf(buffer, format, v);
	return string(buffer);
}

/** An xml writer */
class XmlWriter: public Writer {

///
/// Public interface
///
public:
	XmlWriter(bool indent) : indent(indent) { };
	virtual ~XmlWriter() { };
	
	/** Serializes the object to a string. */
	std::string serialize(bool withClassName, const Writable* w);

///
/// Writer interface
///
public:
	void startWriting(cstr myClassName, unsigned char version = 1) throw (WritingException);
		
		void write_u8(uint8_t, cstr name) throw (WritingException);
		void write_i8(int8_t, cstr name) throw (WritingException);
		void write_i16(int16_t, cstr name) throw (WritingException);
		void write_i32(int32_t, cstr name) throw (WritingException);
		void write_f32(float32_t, cstr name) throw (WritingException);
		void write_f64(float64_t, cstr name) throw (WritingException);
		
		void write_u8(const uint8_t*, size_t n, cstr name) throw (WritingException);
		void write_i8(const int8_t*, size_t n, cstr name) throw (WritingException);
		void write_i16(const int16_t*, size_t n, cstr name) throw (WritingException);
		void write_i32(const int32_t*, size_t n, cstr name) throw (WritingException);
		void write_f32(const float32_t*, size_t n, cstr name) throw (WritingException);
		void write_f64(const float64_t*, size_t n, cstr name) throw (WritingException);

		void writeBytes(const void* i, size_t n, cstr name) throw (WritingException);
		void writeString(cstr s, cstr name) throw (WritingException);
		
		void writeObject(bool needClassName, const Writable* w, cstr name = "") throw (WritingException);
	
	void doneWriting() throw(WritingException);

private:
	bool indent, writeNs;
	xmlTextWriter* writer;
	xmlBuffer* buffer;
	std::string curObjectName;

	/** Utility functions **/
	inline void startElement(const char* type) {
		xmlTextWriterStartElementNS(writer, XML_NAMESPACE_PREFIX, BAD_CAST type, XML_NAMESPACE_URI);
	}
	
	inline void writeAttribute(const char* attributeName, const char* attributeValue) {
		xmlTextWriterWriteAttribute(writer, BAD_CAST attributeName, BAD_CAST attributeValue);
	}
	
	inline void startNamedElement(const char* type, cstr name) {
		startElement(type);
		if (name != "") writeAttribute("name", name.c_str());
	}
	
	inline void endElement() {
		xmlTextWriterEndElement(writer);
	}

	/** Throws an exception, writing also the stack trace. */
	void throwIt(cstr error) throw (WritingException);
};

}}} // namespace

#endif
