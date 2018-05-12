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

#include "binarywriter.h"	
#include "packets.h"
#include "constants.h"
	
namespace RDK2 { namespace Serialization { namespace Binary {
	
	BinaryWriter::~BinaryWriter(){};
	
	Buffer BinaryWriter::createNamePacket(cstr name, Buffer payload) {
		if(!writeNames || (name=="")) return payload;
		Buffer packet;
		packet.push_back(PACKET_HEADER_NAME);
			Buffer namePacket = packet_string(name);
			packet.append(nbo_i32(payload.length()+namePacket.length()));
			packet.append(namePacket);
			packet.append(payload);
		return packet;
	}
	
	void BinaryWriter::write_u8(uint8_t v, cstr name) throw(WritingException) {
		appendBuffer( createNamePacket( name, packet_u8(v) ) );
	}
	void BinaryWriter::write_i8(int8_t v, cstr name) throw(WritingException) {
		appendBuffer( createNamePacket( name, packet_i8(v) ) );
	}
	void BinaryWriter::write_i16(int16_t v,cstr name) throw(WritingException) {
		appendBuffer( createNamePacket( name, packet_i16(v) ) );
	}
	void BinaryWriter::write_i32(int32_t v, cstr name) throw(WritingException) {
		appendBuffer( createNamePacket( name, packet_i32(v) ) );
	}
	void BinaryWriter::write_f32(float32_t v, cstr name)throw(WritingException) {
		appendBuffer( createNamePacket( name, packet_f32(v) ) );
	}
	void BinaryWriter::write_f64(float64_t v, cstr name)throw(WritingException) {
		appendBuffer( createNamePacket( name, packet_f64(v) ) );
	}
	
	void BinaryWriter::writeBytes(const void*v, size_t n, cstr name="") throw(WritingException){
		appendBuffer( createNamePacket(name, packet_bytes(v,n)));
	}
	void BinaryWriter::write_u8(const uint8_t*v, size_t n, cstr name="")throw(WritingException){
		writeBytes(v, n, name);
	}
	void BinaryWriter::write_i8(const int8_t*v, size_t n, cstr name="")throw(WritingException){
		writeBytes(v, n, name);
	}
	void BinaryWriter::write_i16(const int16_t*v, size_t n, cstr name="")throw(WritingException){
		appendBuffer( createNamePacket(name, packet_i16(v,n)));
	}
	void BinaryWriter::write_i32(const int32_t*v, size_t n, cstr name="")throw(WritingException){
		appendBuffer( createNamePacket(name, packet_i32(v,n)));
	}
	void BinaryWriter::write_f32(const float32_t*v, size_t n, cstr name="")throw(WritingException){
		appendBuffer( createNamePacket(name, packet_f32(v,n)));
	}
	void BinaryWriter::write_f64(const float64_t*v, size_t n, cstr name="") throw(WritingException){
		appendBuffer( createNamePacket(name, packet_f64(v,n)));
	}	
	
	void BinaryWriter::writeString(cstr s, cstr name="") throw(WritingException) {
		appendBuffer( createNamePacket(name, packet_string(s)));
	}
	
		
	}}} // end namespace 
