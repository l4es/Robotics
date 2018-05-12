/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007  Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
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

#ifndef H_PACKETS
#define H_PACKETS

#include <string>
#include <rdkcore/serialization/utils.h>

namespace RDK2 { namespace Serialization { namespace Binary {

	typedef std::string Buffer;

	Buffer packet_i8(int8_t c);
	Buffer packet_u8(uint8_t c);
	Buffer packet_i16(int16_t c);
	Buffer packet_i32(int32_t c);
	Buffer packet_f32(float32_t d);
	Buffer packet_f64(float64_t d);
	
	Buffer packet_bytes(const void*, size_t);
	Buffer packet_string(cstr s);
	Buffer packet_i16(const int16_t*, size_t);
	Buffer packet_i32(const int32_t*, size_t);
	Buffer packet_f32(const float32_t*, size_t);
	Buffer packet_f64(const float64_t*, size_t);
	
	Buffer packet_objectWithClass(cstr className, unsigned char version, Buffer payload);
	Buffer packet_objectWithoutClass(Buffer payload);
	
	Buffer nbo_i32(int32_t v);
	
	std::string packetName(char);
	
	}}} // end namespace 

#endif
