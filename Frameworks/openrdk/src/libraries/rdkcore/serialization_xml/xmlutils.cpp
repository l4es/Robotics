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

#include <string.h>
#include <libxml/encoding.h>

#include "xmlutils.h"

xmlChar* xmled(const char* in, bool encode)
{
	xmlChar *out;
	int ret;
	int size;
	int out_size;
	int temp;
	xmlCharEncodingHandlerPtr handler;
	
	if (in == 0)
		return 0;
	
	handler = xmlFindCharEncodingHandler(XML_ENCODING);
	
	if (!handler) {
		printf("ConvertInput: no encoding handler found for '%s'\n",
		XML_ENCODING ? XML_ENCODING : "");
		return 0;
	}
	
	size = (int) strlen(in) + 1;
	out_size = size * 2 - 1;
	out = (unsigned char *) xmlMalloc((size_t) out_size);
	
	if (out != 0) {
		temp = size - 1;
		if (encode) ret = handler->input(out, &out_size, (const xmlChar *) in, &temp);
		else ret = handler->output(out, &out_size, (const xmlChar *) in, &temp);
		if ((ret < 0) || (temp - size + 1)) {
		if (ret < 0) {
			printf("ConvertInput: conversion wasn't successful.\n");
		} else {
			printf
			("ConvertInput: conversion wasn't successful. converted: %i octets.\n",
			temp);
		}
	
		xmlFree(out);
		out = 0;
		} else {
		out = (unsigned char *) xmlRealloc(out, out_size + 1);
		out[out_size] = 0;  /*null terminating out */
		}
	} else {
		printf("ConvertInput: no mem\n");
	}
	
	return out;
}

xmlChar* xmlEncode(const char *in)
{
	return xmled(in, true);
}

xmlChar* xmlDecode(const char *in)
{
	return xmled(in, false);
}

char rstr[] = {
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  62,   0,   0,   0,  63,
	52,  53,  54,  55,  56,  57,  58,  59,  60,  61,   0,   0,   0,   0,   0,   0,
	0,   0,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,
	15,  16,  17,  18,  19,  20,  21,  22,  23,  24,  25,   0,   0,   0,   0,   0,
	0,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,  40,
	41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,   0,   0,   0,   0,   0};

void decode64(const std::string& input, std::string& output, size_t& n)
{
	size_t i = 0;
	size_t l = input.size();
	n = 0;

	output = "";
	while (i < l) {
		while (i < l && (input[i] == 13 || input[i] == 10)) i++;
		if (i < l) {
			char b1 = (char)((rstr[(int)input[i]] << 2 & 0xfc) +
				(rstr[(int)input[i + 1]] >> 4 & 0x03));
			output += b1; n++;
			if (input[i + 2] != '=') {
				char b2 = (char)((rstr[(int)input[i + 1]] << 4 & 0xf0) +
					(rstr[(int)input[i + 2]] >> 2 & 0x0f));
				output += b2; n++;
			}
			if (input[i + 3] != '=') {
				char b3 = (char)((rstr[(int)input[i + 2]] << 6 & 0xc0) +
					rstr[(int)input[i + 3]]);
				output += b3; n++;
			}
			i += 4;
		}
	}
}

const char* bstr =
	"ABCDEFGHIJKLMNOPQ"
	"RSTUVWXYZabcdefgh"
	"ijklmnopqrstuvwxy"
	"z0123456789+/";

void encode64(const char* input, size_t n, std::string& output, bool add_crlf)
{
	size_t i = 0;
	size_t o = 0;

	output = "";
	while (i < n) {
		size_t remain = n - i;
		if (add_crlf && o && o % 76 == 0)
			output += "\n";
		switch (remain) {
		case 1:
			output += bstr[ ((input[i] >> 2) & 0x3f) ];
			output += bstr[ ((input[i] << 4) & 0x30) ];
			output += "==";
			break;
		case 2:
			output += bstr[ ((input[i] >> 2) & 0x3f) ];
			output += bstr[ ((input[i] << 4) & 0x30) + ((input[i + 1] >> 4) & 0x0f) ];
			output += bstr[ ((input[i + 1] << 2) & 0x3c) ];
			output += "=";
			break;
		default:
			output += bstr[ ((input[i] >> 2) & 0x3f) ];
			output += bstr[ ((input[i] << 4) & 0x30) + ((input[i + 1] >> 4) & 0x0f) ];
			output += bstr[ ((input[i + 1] << 2) & 0x3c) + ((input[i + 2] >> 6) & 0x03) ];
			output += bstr[ (input[i + 2] & 0x3f) ];
		}
		o += 4;
		i += 3;
	}
}
