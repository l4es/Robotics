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

#include <rdkcore/object/objectmanager.h>

#include <rdkcore/logging/logging.h>
#include <cstring>
#define LOGGING_MODULE "XmlReader"

#include "xmlreader.h"
#include "xmlutils.h"

namespace RDK2 { namespace Serialization { namespace Xml {

Readable* XmlReader::deserialize(cstr s) throw (ReadingException)
{
	reader = xmlReaderForMemory(s.c_str(), s.size(), 0, XML_ENCODING, 0);
	XML_CHECK_ERROR(!reader, "Cannot create xmlTextReader", 0);

	innerObjects = 0;
	Readable* r = readObject();

	xmlFreeTextReader(reader);
	return r;
}

void XmlReader::deserialize(cstr s, Readable* readable) throw (ReadingException)
{
	reader = xmlReaderForMemory(s.c_str(), s.size(), 0, XML_ENCODING, 0);
	XML_CHECK_ERROR(!reader, "Cannot create xmlTextReader", );

	innerObjects = 0;
	readObject(readable);

	xmlFreeTextReader(reader);
}

void XmlReader::iWantAnElement(cstr elementName, bool dontCareIfEmpty) throw (ReadingException)
{
	if (!goToElementOrEndElement()) {
		throwIt(string() + "I was looking for the element '" + elementName + "' but the stream has finished");
	}
	if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT) {
		const char* xs = (const char*) xmlTextReaderConstName(reader);
		throwIt(string() + "I was looking for the element '" + elementName 
			+ "' but I found the end of the element '" + string(xs ? xs : "") + "'");
	}
	if (xmlStrcmp(BAD_CAST elementName.c_str(), xmlTextReaderConstName(reader)) != 0) {
		const char* xs = (const char*) xmlTextReaderConstName(reader);
		throwIt(string() + "I was looking for the element '" + elementName 
			+ "' but I found the element '" + string(xs ? xs : "") + "'");
	}
	if (!dontCareIfEmpty) {
		if (xmlTextReaderIsEmptyElement(reader)) currentElementIsEmpty = true;
		else currentElementIsEmpty = false;
	}
}

void XmlReader::iWantANamedElement(cstr elementName, cstr name, bool dontCareIfEmpty) throw (ReadingException)
{
	try {
		iWantAnElement(elementName, dontCareIfEmpty);
	} catch (ReadingException e) {
		// We tell the name of the attribute: it's useful for
		// debugging
		RDK_ERROR_STREAM("Unable to read element named '" << name
				 << "'");
		throw e;
	}
	if (name != "" && getAttributeValue("name") != name) {
		throwIt(string() + "Expected element with name '" + name + "', got '" + getAttributeValue("name") + "'");
	}
}

std::string XmlReader::getAttributeValue(cstr attributeName) throw (ReadingException)
{
	xmlChar* tmp = xmlTextReaderGetAttribute(reader, BAD_CAST attributeName.c_str());
	string s((tmp ? (const char*) tmp : ""));
	xmlFree(tmp);
	return s;
}

void XmlReader::iWantAnEndElement(cstr elementName) throw (ReadingException)
{
	if (currentElementIsEmpty) return;
	if (!goToElementOrEndElement()) {
		throwIt(string() + "I was looking for the end of element '" + elementName + "' but the stream has finished");
	}
	if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT) {
		const char* xs = (const char*) xmlTextReaderConstName(reader);
		throwIt(string() + "I was looking for the end of element '" + elementName 
			+ "' but I found the element '" + string(xs ? xs : "") + "'");
	}
	if (xmlStrcmp(BAD_CAST elementName.c_str(), xmlTextReaderConstName(reader)) != 0) {
		const char* xs = (const char*) xmlTextReaderConstName(reader);
		throwIt(string() + "I was looking for the end of element '" + elementName 
			+ "' but I found the end of element '" 
			+ string(xs ? xs : "") + "'");
	}
}

bool XmlReader::goToElementOrEndElement()
{
	if (alreadyRead) {
		alreadyRead = false;
		return true;
	}
	if (checkBeforeGoingToNext && 
	(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT
	|| xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)) {
		checkBeforeGoingToNext = false;
		return true;
	}
	int r = 1;
	while ((r = xmlTextReaderRead(reader)) > 0
	&& xmlTextReaderNodeType(reader) != XML_READER_TYPE_ELEMENT
	&& xmlTextReaderNodeType(reader) != XML_READER_TYPE_END_ELEMENT) ;
	checkBeforeGoingToNext = false;
	return r > 0;
}

unsigned char XmlReader::startReading(cstr className) throw (ReadingException)
{
	iWantANamedElement("rdk:object", curObjectName, false);
	if (getAttributeValue("class") != className) 
		throwIt(string() + "Wrong class found, expected '" + className + 
			"', got '" + getAttributeValue("class") + "'");
	innerObjects++;
	string verStr = getAttributeValue("version");
	int vers = (verStr == "") ? 1 : atoi(verStr.c_str());
	return (unsigned char) vers;
}

uint8_t XmlReader::read_u8(cstr name) throw (ReadingException)
{
	iWantANamedElement("rdk:uint8", name);
	return atoi(getAttributeValue("value").c_str());
}

int8_t XmlReader::read_i8(cstr name) throw (ReadingException)
{
	iWantANamedElement("rdk:int8", name);
	return atoi(getAttributeValue("value").c_str());
}

int16_t XmlReader::read_i16(cstr name) throw (ReadingException)
{
	iWantANamedElement("rdk:int16", name);
	return atoi(getAttributeValue("value").c_str());
}

int32_t XmlReader::read_i32(cstr name) throw (ReadingException)
{
	iWantANamedElement("rdk:int32", name);
	return atoi(getAttributeValue("value").c_str());
}

float32_t XmlReader::read_f32(cstr name) throw (ReadingException)
{
	iWantANamedElement("rdk:float32", name);
	return atof(getAttributeValue("value").c_str());
}

float64_t XmlReader::read_f64(cstr name) throw (ReadingException)
{
	iWantANamedElement("rdk:float64", name);
	return atof(getAttributeValue("value").c_str());
}

void XmlReader::readBytes(void** buffer, size_t* n, cstr name) throw (ReadingException)
{
	iWantANamedElement("rdk:buffer64", name, false);
	xmlTextReaderRead(reader);
	string s;
	size_t nn;
	const char* xs = (const char*) xmlTextReaderConstValue(reader);
	decode64(string(xs ? xs : ""), s, nn);
	iWantAnEndElement("rdk:buffer64");
	
	*buffer = new char[s.size()];
	memcpy(*buffer, s.data(), s.size());
	*n = nn;
}

void XmlReader::iWantANamedElementArrayOfType(cstr name, cstr type) throw (ReadingException)
{
	iWantANamedElement("rdk:array", name);
	if (getAttributeValue("type") != type) {
		throwIt(string() + "I expected an array of " + type +
			", but got an array of " + getAttributeValue("type"));
	}
}

void XmlReader::readArrayOfBytes(uint8_t** a, size_t *n) throw (ReadingException)
{
	size_t sz = atoi(getAttributeValue("size").c_str());
	xmlTextReaderRead(reader);
	if (sz == 0) throwIt("No 'size' attribute in array element");
	*a = new uint8_t[sz];
	const char* xs = (const char*) xmlTextReaderConstValue(reader);
	stringstream ss(string(xs ? xs : ""));
	for (size_t i = 0; i < sz; i++) {
		int q;
		bool b = (ss >> q);
		(*a)[i] = (uint8_t) q;
		if (!b) 
			throwIt(string() + "I expected more array values (string was: '" 
				+ ss.str() + "')");
	}
	*n = sz;
}

void XmlReader::read_i8(int8_t** a, size_t* n, cstr name) throw (ReadingException)
{
	iWantANamedElementArrayOfType(name, "int8");
	readArrayOfBytes((uint8_t**) a, n);
	for (size_t i = 0; i < *n; i++) cerr << (*a)[i] << " ";
	cerr << endl;
	iWantAnEndElement("rdk:array");
}

void XmlReader::read_u8(uint8_t** a, size_t* n, cstr name) throw (ReadingException)
{
	iWantANamedElementArrayOfType(name, "uint8");
	readArrayOfBytes(a, n);
	iWantAnEndElement("rdk:array");
}

void XmlReader::read_i16(int16_t** a, size_t* n, cstr name) throw (ReadingException)
{
	iWantANamedElementArrayOfType(name, "int16");
	readArray(a, n);
	iWantAnEndElement("rdk:array");
}

void XmlReader::read_i32(int32_t** a, size_t* n, cstr name) throw (ReadingException)
{
	iWantANamedElementArrayOfType(name, "int32");
	readArray(a, n);
	iWantAnEndElement("rdk:array");
}

void XmlReader::read_f32(float32_t** a, size_t* n, cstr name) throw (ReadingException)
{
	iWantANamedElementArrayOfType(name, "float32");
	readArray(a, n);
	iWantAnEndElement("rdk:array");
}

void XmlReader::read_f64(float64_t** a, size_t* n, cstr name) throw (ReadingException)
{
	iWantANamedElementArrayOfType(name, "float64");
	readArray(a, n);
	iWantAnEndElement("rdk:array");
}


// FIXME 
// In questo caso:
//
//    <rdk:object class='RString' xmlns:rdk='http://sied.dis.uniroma1.it/rdk'>                         
//    	<rdk:string/>                                                                                  
//    </rdk:object>
//
// questa funzione restituisce "\n" e non "", infatti non controlla se l'elemento
// rdk:string e' vuoto e passa al frammento di testo successivo (fra rdk:string 
// e rdk:object)

std::string XmlReader::readString(cstr name) throw (ReadingException)
{
	iWantANamedElement("rdk:string", name, false);
	xmlTextReaderRead(reader);
	const char* cv = (const char*) xmlTextReaderConstValue(reader);
	if(!cv) {
		// stringa vuota
		return "";
	}
	xmlChar* encstr = xmlDecode(cv);
	if(!encstr) {
		RDK_ERROR_STREAM("xmlDecode failed");
		throwIt("could not decode string (xmlDecode failed)");	
	}
	
	string s((const char*) encstr);
	xmlFree(encstr);
	iWantAnEndElement("rdk:string");
	return s;
}

void XmlReader::readObject(Readable* r, cstr name) throw (ReadingException)
{
	curObjectName = name;
	int curInnerObject = innerObjects;
	r->read(this);
	if (innerObjects != curInnerObject) {
		throwIt("The object did not call doneReading()");
	}
}

Readable* XmlReader::readObject(cstr name) throw (ReadingException)
{
	iWantANamedElement("rdk:object", name);
	alreadyRead = true;
	Readable* r = RDK2::Meta::forName(getAttributeValue("class").c_str()); //instantiate
	if (!r) {
		throwIt(string() + "Could not instantiate object of class " + getAttributeValue("class"));
	}
	readObject(r, name);
	return r;
}

void XmlReader::doneReading() throw (ReadingException)
{
	iWantAnEndElement("rdk:object");
	innerObjects--;
}

void XmlReader::throwIt(cstr errorMessage) throw (ReadingException)
{
	throw ReadingException(errorMessage);
}

}}} // namespace
