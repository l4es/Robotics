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

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "XmlWriter"

#include "xmlwriter.h"
#include "xmlutils.h"

namespace RDK2 { namespace Serialization { namespace Xml {

string XmlWriter::serialize(bool withClassName, const Writable* w)
{
	buffer = xmlBufferCreate();
	XML_CHECK_ERROR(!buffer, "Cannot create xmlBuffer", "");

	writer = xmlNewTextWriterMemory(buffer, 0);
	XML_CHECK_ERROR(!writer, "Cannot create xmlTextWriter", "");

	if (indent) {
		xmlTextWriterSetIndent(writer, 1);
		xmlTextWriterSetIndentString(writer, BAD_CAST XML_INDENTATION_STRING);
	}
	else {
		xmlTextWriterSetIndent(writer, 0);
	}

	int r;
	r = xmlTextWriterStartDocument(writer, 0, XML_ENCODING, 0);
	XML_CHECK_ERROR(r < 0, "Cannot start xml document", "");

	writeNs = true;
	writeObject(withClassName, w);

	xmlTextWriterEndDocument(writer);
	xmlTextWriterFlush(writer);
	string rs((const char*) buffer->content);
	xmlFreeTextWriter(writer);
	xmlBufferFree(buffer);
	return rs;
}

void XmlWriter::startWriting(cstr myClassName, unsigned char version) throw (WritingException)
{
	xmlTextWriterStartElementNS(writer, XML_NAMESPACE_PREFIX, BAD_CAST "object",
		writeNs ? writeNs = false, XML_NAMESPACE_URI_ROOT : XML_NAMESPACE_URI);
	if (curObjectName != "") writeAttribute("name", curObjectName.c_str());
	writeAttribute("class", myClassName.c_str());
	char buf[16];
	sprintf(buf, "%d", version);
	if (version != 1) writeAttribute("version", buf);
}

void XmlWriter::write_u8(uint8_t v, cstr name) throw (WritingException)
{
	startNamedElement("uint8", name);
		writeAttribute("value", vtos(v, XML_INTEGER_VALUE_PRINTF_FORMAT).c_str());
	endElement();
}

void XmlWriter::write_i8(int8_t v, cstr name) throw (WritingException)
{
	startNamedElement("int8", name);
		writeAttribute("value", vtos(v, XML_INTEGER_VALUE_PRINTF_FORMAT).c_str());
	endElement();
}

void XmlWriter::write_i16(int16_t v, cstr name) throw (WritingException)
{
	startNamedElement("int16", name);
		writeAttribute("value", vtos(v, XML_INTEGER_VALUE_PRINTF_FORMAT).c_str());
	endElement();
}

void XmlWriter::write_i32(int32_t v, cstr name) throw (WritingException)
{
	startNamedElement("int32", name);
		writeAttribute("value", vtos(v, XML_INTEGER_VALUE_PRINTF_FORMAT).c_str());
	endElement();
}

void XmlWriter::write_f32(float32_t v, cstr name) throw (WritingException)
{
	startNamedElement("float32", name);
		writeAttribute("value", vtos(v, XML_FLOATING_VALUE_PRINTF_FORMAT).c_str());
	endElement();
}

void XmlWriter::write_f64(float64_t v, cstr name) throw (WritingException)
{
	startNamedElement("float64", name);
		writeAttribute("value", vtos(v, XML_FLOATING_VALUE_PRINTF_FORMAT).c_str());
	endElement();
}

void XmlWriter::writeBytes(const void* buf, size_t n, cstr name) throw (WritingException)
{
	startNamedElement("buffer64", name);
		// we SHOULD use this function, if there were the 
		// xmlTextWriterWriteBase64(writer, (const char*) buf, 0, n);
		string s;
		encode64((const char*) buf, n, s, true);
		xmlTextWriterWriteString(writer, BAD_CAST s.c_str());
	endElement();
}

void XmlWriter::write_i8(const int8_t* v, size_t n, cstr name) throw (WritingException)
{
	startNamedElement("array", name);
		writeAttribute("type", "int8");
		writeAttribute("size", vtos(n, "%d").c_str());
		for (size_t i = 0; i < n; i++) {
			xmlTextWriterWriteFormatString(writer, XML_INTEGER_VALUE_PRINTF_FORMAT, v[i]);
			if (i != n - 1) xmlTextWriterWriteString(writer, BAD_CAST " ");
		}
	endElement();
}

void XmlWriter::write_u8(const uint8_t* v, size_t n, cstr name) throw (WritingException)
{
	startNamedElement("array", name);
		writeAttribute("type", "uint8");
		writeAttribute("size", vtos(n, "%d").c_str());
		for (size_t i = 0; i < n; i++) {
			xmlTextWriterWriteFormatString(writer, XML_INTEGER_VALUE_PRINTF_FORMAT, v[i]);
			if (i != n - 1) xmlTextWriterWriteString(writer, BAD_CAST " ");
		}
	endElement();
}

void XmlWriter::write_i16(const int16_t* v, size_t n, cstr name) throw (WritingException)
{
	startNamedElement("array", name);
		writeAttribute("type", "int16");
		writeAttribute("size", vtos(n, "%d").c_str());
		for (size_t i = 0; i < n; i++) {
			xmlTextWriterWriteFormatString(writer, XML_INTEGER_VALUE_PRINTF_FORMAT, v[i]);
			if (i != n - 1) xmlTextWriterWriteString(writer, BAD_CAST " ");
		}
	endElement();
}

void XmlWriter::write_i32(const int32_t* v, size_t n, cstr name) throw (WritingException)
{
	startNamedElement("array", name);
		writeAttribute("type", "int32");
		writeAttribute("size", vtos(n, "%d").c_str());
		for (size_t i = 0; i < n; i++) {
			xmlTextWriterWriteFormatString(writer, XML_INTEGER_VALUE_PRINTF_FORMAT, v[i]);
			if (i != n - 1) xmlTextWriterWriteString(writer, BAD_CAST " ");
		}
	endElement();
}

void XmlWriter::write_f32(const float32_t* v, size_t n, cstr name) throw (WritingException)
{
	startNamedElement("array", name);
		writeAttribute("type", "float32");
		writeAttribute("size", vtos(n, "%d").c_str());
		for (size_t i = 0; i < n; i++) {
			xmlTextWriterWriteFormatString(writer, XML_FLOATING_VALUE_PRINTF_FORMAT, v[i]);
			if (i != n - 1) xmlTextWriterWriteString(writer, BAD_CAST " ");
		}
	endElement();
}

void XmlWriter::write_f64(const float64_t* v, size_t n, cstr name) throw (WritingException)
{
	startNamedElement("array", name);
		writeAttribute("type", "float64");
		writeAttribute("size", vtos(n, "%d").c_str());
		for (size_t i = 0; i < n; i++) {
			xmlTextWriterWriteFormatString(writer, XML_FLOATING_VALUE_PRINTF_FORMAT, v[i]);
			if (i != n - 1) xmlTextWriterWriteString(writer, BAD_CAST " ");
		}
	endElement();
}

void XmlWriter::writeString(cstr s, cstr name) throw (WritingException)
{
	startElement("string");
		writeAttribute("name", name.c_str());
		xmlChar* encstr = xmlEncode(s.c_str());
		xmlTextWriterWriteString(writer, encstr);
		xmlFree(encstr);
	endElement();
}

// NOTE: XmlWriter::writeObject writes ALWAYS the className of the object
void XmlWriter::writeObject(bool /* see note above */, const Writable* w, cstr name) throw (WritingException)
{
	curObjectName = name;
	w->write(this);
}

void XmlWriter::doneWriting() throw (WritingException)
{
	xmlTextWriterEndElement(writer);
}

void XmlWriter::throwIt(cstr errorMessage) throw (WritingException)
{
	throw WritingException(errorMessage);
}

}}} // namespace
