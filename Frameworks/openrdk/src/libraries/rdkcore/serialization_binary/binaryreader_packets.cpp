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

#include "binaryreader.h"
#include "constants.h"
#include "packets.h"

namespace RDK2 { namespace Serialization { namespace Binary {
			
	void BinaryReader::expectPacket(char expected, std::string message)
		throw(ReadingException)
	{
		int8_t header = stream_read_int8();
		if(header!=expected) {
			throwIt(
				message+": expected '" + packetName(expected) + "', read '" + packetName(header) + "'."); 	
		}
	}
	
	bool BinaryReader::checkName(cstr name) throw(ReadingException) {
		int8_t next = stream_peek_int8(); 
		if(next == PACKET_HEADER_NAME) {
			BINARY_READER_DEBUG("Read name packet");
			stream_read_int8();
			enterContext(Context::ReadingName);
			context().name_readName = stream_read_string();
			if( (name != "") && (name!=context().name_readName ) ) {
				throwIt(string("Name error: looking for ") + 
					name + ", instead I read " + context().name_readName );
			}
			return true;
		} else {
			BINARY_READER_DEBUG("Next is not a name packet: " << packetName(next));
			return false;
		}
	}
	
	uint8_t BinaryReader::read_u8 (cstr name) throw(ReadingException){
		assertValidRead("read_u8");
		bool present = checkName(name);
		expectPacket(PACKET_HEADER_INT8, "Could not read int8 packet");
		int8_t value = stream_read_int8();
		if(present) exitContext();
		return *(uint8_t*)&value;
	}
	
	int8_t  BinaryReader::read_i8 (cstr name) throw(ReadingException){
		assertValidRead("read_i8");
		bool present = checkName(name);
		expectPacket(PACKET_HEADER_INT8, "Could not read int8 packet");
		int8_t value = stream_read_int8();
		if(present) exitContext();
		return value;
	}
	
	int16_t BinaryReader::read_i16(cstr name) throw(ReadingException){
		assertValidRead("read_i16");
		bool present = checkName(name);
		expectPacket(PACKET_HEADER_INT16, "Could not read int16 packet");
		int16_t value = stream_read_int16();
		if(present) exitContext();
		return value;
	}
	
	int32_t BinaryReader::read_i32(cstr name) throw(ReadingException){
		assertValidRead("read_i32");
		bool present = checkName(name);
		BINARY_READER_DEBUG("Reading int32, name is present = " << present);
		expectPacket(PACKET_HEADER_INT32, "Could not read int32 packet");
		int32_t value = stream_read_int32();	
		if(present) exitContext();
		return value;
	}
	
	float32_t BinaryReader::read_f32(cstr name) throw(ReadingException){
		assertValidRead("read_f32");
		bool present = checkName(name);
		expectPacket(PACKET_HEADER_FLOAT32, "Could not read float32 packet");
		float32_t value = stream_read_float32();	
		if(present) exitContext();
		return value;
	}
	
	float64_t BinaryReader::read_f64(cstr name) throw(ReadingException){
		assertValidRead("read_f64");
		bool present = checkName(name);
		expectPacket(PACKET_HEADER_FLOAT64, "Could not read float64 packet");
		float64_t value = stream_read_float64();	
		if(present) exitContext();
		return value;
	}
	
	void BinaryReader::readBytes(void** buf, size_t* n, cstr name) throw (ReadingException) {
		read_i8((int8_t**)buf, n, name);
	}

	void BinaryReader::read_u8(uint8_t** u, size_t* n, cstr name) throw (ReadingException) {
		read_i8((int8_t**)u, n, name);
	}

	void BinaryReader::read_i8(int8_t** i,    size_t* n, cstr name) throw(ReadingException) {
		assertValidRead("read_i8[");
		bool present = checkName(name);
		expectPacket(PACKET_HEADER_ARRAY_INT8, "Could not read byte array packet");
		*n = stream_read_int32();
		
		//if(*n==0) 
		//	throwIt("Zero length array?");
		
		int8_t * buffer = new int8_t[*n];
		for(int a=0;a<(int)*n;a++)
			buffer[a] = stream_read_int8();
		
		*i = buffer;
		if(present) exitContext();
	}
	
	void BinaryReader::read_i16  (int16_t**i,   size_t* n, cstr name) throw(ReadingException) {
		assertValidRead("read_i16[");
		bool present = checkName(name);
		expectPacket(PACKET_HEADER_ARRAY_INT16, "Could not read int16 array packet");
		*n = stream_read_int32();
		
		//if(*n==0) 
		//	throwIt("Zero length array?");
		
		int16_t * buffer = new int16_t[*n];
		for(int a=0;a<(int)*n;a++)
			buffer[a] = stream_read_int16();
		
		*i = buffer;
		if(present) exitContext();
	}
	
	void BinaryReader::read_i32  (int32_t**i,   size_t* n, cstr name) throw(ReadingException) {
		assertValidRead("read_i32[");
		bool present = checkName(name);
		expectPacket(PACKET_HEADER_ARRAY_INT32, "Could not read int32 array packet");
		*n = stream_read_int32();
		
		//printf("N: %l ",*n);
		//if(*n==0) 
		//	throwIt("Zero length array?");
		
		int32_t * buffer = new int32_t[*n];
		for(int a=0;a<(int)*n;a++)
			buffer[a] = stream_read_int32();
		
		*i = buffer;
		if(present) exitContext();
	}
	
	void BinaryReader::read_f32  (float32_t**i, size_t* n, cstr name) throw(ReadingException) {
		assertValidRead("read_f32[");
		bool present = checkName(name);
		expectPacket(PACKET_HEADER_ARRAY_FLOAT32, "Could not read float32 array packet");
		*n = stream_read_int32();
		
		//if(*n==0) 
		//	throwIt("Zero length array?");
		
		float32_t * buffer = new float32_t[*n];
		for(int a=0;a<(int)*n;a++)
			buffer[a] = stream_read_float32();
		
		*i = buffer;
		
		if(present) exitContext();
	}
	
	void BinaryReader::read_f64  (float64_t**i, size_t* n, cstr name) throw(ReadingException){
		assertValidRead("read_f64[");
		bool present = checkName(name);
		expectPacket(PACKET_HEADER_ARRAY_FLOAT64, "Could not read float64 array packet");
		*n = stream_read_int32();
		
		//if(*n==0) 
		//	throwIt("Zero length array?");
		
		float64_t * buffer = new float64_t[*n];
		for(int a=0;a<(int)*n;a++)
			buffer[a] = stream_read_float64();
		
		*i = buffer;
		
		if(present) exitContext();
	}
	
		
	std::string BinaryReader::readString(cstr name) throw(ReadingException) {
		assertValidRead("read_string");
		bool present = checkName(name);
		std::string s = stream_read_string();
		if(present) exitContext();
		return s;
	}
	
	std::string BinaryReader::stream_read_string() throw (ReadingException) {
		expectPacket(PACKET_HEADER_STRING, "Could not read string packet");
		int length = stream_read_int32();
		
		if(length==0) return "";
		
		std::string s; s.reserve(length);
		
		for(int a=0;a<(int)length;a++)
			s.push_back(stream_read_int8());
		
		BINARY_READER_DEBUG("stream: assembled string, length="<<s.length()<<" s="<<s);
		return s;
	}
	
	
}}} // end namespace 
