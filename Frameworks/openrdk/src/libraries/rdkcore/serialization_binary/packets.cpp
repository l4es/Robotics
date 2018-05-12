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


#include "packets.h"
#include "constants.h"
#include "assert.h"

#include <rdkcore/geometry/utils.h>
#include <rdkcore/rdkmath/rdkmath.h>

#include <float.h>

#ifdef LINUX
      #include <netinet/in.h>
#endif
#ifdef CYGWIN
      #include <netinet/in.h>
#endif

using namespace RDK2::Geometry;

namespace RDK2 { namespace Serialization { namespace Binary {
			
		Buffer nbo_i16(int16_t v){
			uint16_t network = htons( v );
			return Buffer( (char*)&network, 2);
		}
		Buffer nbo_i32(int32_t v){
			uint32_t network = htonl( v );
			return Buffer( (char*)&network, 4);
		}
		
		Buffer nbo_f32(float32_t v){
			volatile float32_t a = v;
			volatile uint32_t network = htonl(*reinterpret_cast<volatile uint32_t*>(&a));
			return Buffer( (char*)&network, 4);
		}

/*
		Buffer nbo_f64(float64_t v){
			char network[8];
			// non sono proprio sicuro di questo
			((int32_t*) network)[0] = htonl( ((int32_t*)&v)[0]);  
			((int32_t*) network)[1] = htonl(((int32_t*)&v)[1]);
			return Buffer( network, 8);
		}
*/

	/// Note: please make sure it is in sync with the companion in binaryreader_bytes.cpp
		Buffer nbo_f64(float64_t v){
			Buffer p; 
				if(v == DBL_MAX) {
					float32_t a = FLT_MAX;
					float32_t b = 0;
					p.append(nbo_f32(a));
					p.append(nbo_f32(b));
				} else if( RDK_ISINF(v) ) {
						float32_t a = RDK_INF32;
						float32_t b = 0;
						p.append(nbo_f32(a));
						p.append(nbo_f32(b));
				} else {
					float32_t a = (float32_t) v;
					float32_t b = (float32_t) (v-a);
					p.append(nbo_f32(a));
					p.append(nbo_f32(b));
				}
			return p;
		}

		
		
		Buffer packet_i8(int8_t c) {
			Buffer packet;
				packet.push_back(PACKET_HEADER_INT8);
				packet.push_back(c);	
			return packet;
		}
		
		Buffer packet_u8(uint8_t c) {
			Buffer packet;
				packet.push_back(PACKET_HEADER_INT8);
				packet.push_back(c); /// XXX 
			return packet;
		}
		
		Buffer packet_i16(int16_t c) {
				Buffer packet;
				packet.push_back(PACKET_HEADER_INT16);
				packet.append(nbo_i16(c));
			return packet;
		}
		
		Buffer packet_i32(int32_t c) {
			Buffer packet;
				packet.push_back(PACKET_HEADER_INT32);
				packet.append(nbo_i32(c));
			return packet;
		}
		
		
		Buffer packet_f32(float32_t d) {
				Buffer packet;
				packet.push_back(PACKET_HEADER_FLOAT32);
				packet.append(nbo_f32(d) );
			return packet;
		}
		
		Buffer packet_f64(float64_t d) {
			Buffer packet;
				packet.push_back(PACKET_HEADER_FLOAT64);
				packet.append(nbo_f64(d)); 
			return packet;
		}
		
		Buffer packet_bytes(const void*v, size_t n) {
			Buffer packet;
				packet.push_back(PACKET_HEADER_ARRAY_INT8);
				packet.append(nbo_i32(n));
				packet.append( Buffer( (const char*) v, n) );
			return packet;
		}
		
		Buffer packet_string(cstr s) {
			Buffer packet;
			packet.push_back(PACKET_HEADER_STRING);
			packet.append(nbo_i32(s.length()));
			if(s.length()>0)
				packet.append(s);
			return packet;
		}
		
		Buffer packet_objectWithClass(cstr className, unsigned char version, Buffer payload){
			Buffer name = packet_string(className);
			int length = name.length() + payload.length() + 1;
			Buffer packet;
			packet.push_back( PACKET_HEADER_OBJECTWITHCLASS );
			packet.append( nbo_i32(length) );
			packet.append( 1, *((char*)&version) );
			packet.append( name );
			packet.append( payload );
			return packet;
		}

		
		Buffer packet_objectWithoutClass(Buffer payload){
			int length = payload.length();
			Buffer packet;
			packet.push_back( PACKET_HEADER_OBJECTNOCLASS );
			packet.append( nbo_i32(length) );
			packet.append( payload );
			return packet;
		}
		
		Buffer packet_i16(const int16_t*v, size_t n) {
			Buffer packet;
			packet.push_back( PACKET_HEADER_ARRAY_INT16);
			packet.append( nbo_i32(n) );
			for(size_t i=0;i<n;i++)
				packet.append(nbo_i16(v[i]));
			
			assert(packet.length() == (size_t) 1+4+2*n );
			return packet;
		}
		
		Buffer packet_i32(const int32_t*v, size_t n){
			Buffer packet;
			packet.push_back( PACKET_HEADER_ARRAY_INT32);
			packet.append( nbo_i32(n) );
			for(size_t i=0;i<n;i++)
				packet.append(nbo_i32(v[i]));
			
			assert(packet.length() == (size_t) 1+4+4*n );
			return packet;
		}
		
		Buffer packet_f32(const float32_t*v, size_t n){
			Buffer packet;
			packet.push_back( PACKET_HEADER_ARRAY_FLOAT32);
			packet.append( nbo_i32(n) );
			for(size_t i=0;i<n;i++)
				packet.append(nbo_f32(v[i]));
			
			assert(packet.length() == (size_t)  1+4+4*n );
			return packet;
		}
		
		Buffer packet_f64(const float64_t*v, size_t n){
			Buffer packet;
			packet.push_back( PACKET_HEADER_ARRAY_FLOAT64);
			packet.append( nbo_i32(n) );
			for(size_t i=0;i<n;i++)
				packet.append(nbo_f64(v[i]));
			
			assert(packet.length() == (size_t) 1+4+8*n );
			return packet;
		}
		
		
	std::string packetName(char c) {
		#define CASE(x) case(x): { return #x ; }
		switch(c) {
			
			CASE(PACKET_HEADER_NAME)
			CASE(PACKET_HEADER_OBJECTWITHCLASS)
			CASE(PACKET_HEADER_OBJECTNOCLASS)
			CASE(PACKET_HEADER_OBJECTARRAY)
			CASE(PACKET_HEADER_INT8)
			CASE(PACKET_HEADER_INT16)
			CASE(PACKET_HEADER_INT32)
			CASE(PACKET_HEADER_FLOAT32)
			CASE(PACKET_HEADER_FLOAT64)
			CASE(PACKET_HEADER_ARRAY_INT8)
			CASE(PACKET_HEADER_ARRAY_INT16)
			CASE(PACKET_HEADER_ARRAY_INT32)
			CASE(PACKET_HEADER_ARRAY_FLOAT32)
			CASE(PACKET_HEADER_ARRAY_FLOAT64)
			
			default:
				string s("(unknown packet '"); s+=c; s+="')";
				return s;
		}
	}

	}}}
