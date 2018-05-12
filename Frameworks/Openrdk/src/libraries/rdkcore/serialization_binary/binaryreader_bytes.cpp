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
#include "packets.h"

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
			
		int8_t    BinaryReader::stream_peek_int8()    throw (ReadingException) { 
			return real_peek_int8(); 
		};

		int8_t    BinaryReader::stream_read_int8()    throw (ReadingException) { 
			advance(1); return real_read_int8(); 
		};
		
		int16_t   BinaryReader::stream_read_int16()   throw (ReadingException) { 
			advance(2); return real_read_int16(); 
		}
		
		int32_t   BinaryReader::stream_read_int32()   throw (ReadingException) { 
			advance(4); return real_read_int32(); 
		}
		
		float32_t BinaryReader::stream_read_float32() throw (ReadingException) { 
			advance(4); return real_read_float32(); 
		}
		
		float64_t BinaryReader::stream_read_float64() throw (ReadingException) { 
			advance(8); return real_read_float64(); 
		}
		
	void BinaryReader::checkInputStream() throw (ReadingException) {
		if(is->fail()) throwIt("Bad input stream");
	}
	
	int8_t BinaryReader::real_peek_int8() throw (ReadingException)  {
		int8_t c = is->peek();
		checkInputStream();
		return c;
	}
	
	int8_t BinaryReader::real_read_int8() throw (ReadingException)  {
		int8_t c;
		is->read( (char*)&c, 1);
		checkInputStream();	
		
		BINARY_READER_DEBUG("stream:  int8: "<<(char)c);
		
		return c;
	}
	
	int16_t BinaryReader::real_read_int16() throw (ReadingException) {
		int16_t v; 
		is->read( (char*) &v, 2);
		checkInputStream();
		return ntohs(v);
	}
	
	int32_t BinaryReader::real_read_int32() throw (ReadingException) {
		int32_t n; 
		is->read( (char*) &n, 4);
		checkInputStream();
		int32_t h = ntohl(n);
		BINARY_READER_DEBUG("stream: int32: "<<h); 
		return h;
	}
	
	float32_t BinaryReader::real_read_float32() throw (ReadingException) {
		uint32_t v; is->read( (char*) &v, 4);
		//printf("Reading uint32: v = %u ", v);
		v = ntohl(v);
		//printf(" ntohl = %u", v);
		checkInputStream();
		float32_t f = *reinterpret_cast<float32_t*>(&v);
		//printf(" float value: %.2f\n", f);
		return f;
	}
	
	/*
	float64_t BinaryReader::real_read_float64() throw (ReadingException) {
		char network[8];
		is->read(network, 8);
		checkInputStream();
		// non sono proprio sicuro di questo
		((int32_t*) network)[0] = ntohl(((int32_t*)network)[0]);  
		((int32_t*) network)[1] = ntohl(((int32_t*)network)[1]);
		return *(float64_t*)network;
	}*/

	/// Note: please make sure it is in sync with the companion in packats.cpp
	float64_t BinaryReader::real_read_float64() throw (ReadingException) {
		float64_t a = real_read_float32();
		float64_t b = real_read_float32();
		if(a == RDK_INF32) return RDK_INF64;
		if(a == FLT_MAX) return DBL_MAX;
		return a+b;
	}
	
	}}} // namespace RDK2
