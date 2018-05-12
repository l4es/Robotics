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

#ifndef RDK2_SERIALIZATION_XML_UTILS
#define RDK2_SERIALIZATION_XML_UTILS

#include <string>
#include <libxml/xmlreader.h>

#define XML_CHECK_ERROR(assertion, msg, retonerr) \
	if ((assertion)) { \
		xmlErrorPtr e = xmlGetLastError(); \
		RDK_ERROR_STREAM(string(msg) << (e ? e->message : "")); \
		return retonerr; \
	}

#define XML_ENCODING "ISO-8859-1"
#define XML_NAMESPACE_PREFIX BAD_CAST "rdk"
#define XML_NAMESPACE_URI_ROOT BAD_CAST "http://sied.dis.uniroma1.it/rdk"
#define XML_NAMESPACE_URI 0

xmlChar* xmlEncode(const char* s);
xmlChar* xmlDecode(const char* s);

void encode64(const char* input, size_t n, std::string& output, bool add_crlf);
void decode64(const std::string& input, std::string& output, size_t& n);

#endif
