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

#ifndef H_BINARY_READER
#define H_BINARY_READER

#include <vector>
#include <iostream>

#include <rdkcore/serialization/read.h>

#define BINARY_READER_ABORT_ON_EXCEPTION 0
#define BINARY_READER_DEBUG_ACTIVE 0

#if BINARY_READER_DEBUG_ACTIVE
	#define BINARY_READER_DEBUG(a) cerr << "breader: " << __FILE__ << ":" << __LINE__ << " " << a << endl;
#else
	#define BINARY_READER_DEBUG(a) 
#endif

namespace RDK2 { namespace Serialization { namespace Binary {

	/** 
	* Here it is the algorithm for dealing with names:
	* If (next is a name packet AND name is specified AND names are different)
	*    throw exception.
	* If (next is a name packet AND name is not specified) 
	*    ignore name packet;
	* If (next is not name packet) 
	*   ignore name parameter.
	*/
	class BinaryReader: public Reader {
		public:
			BinaryReader();
			virtual ~BinaryReader();
		
			Readable * deserialize(cstr);
			void deserialize(cstr, Readable *);
			
		///
		/// Reader interface
		///
		public:
			/** If next packet is not of specified class name, throws exception. */
			unsigned char startReading(cstr className) throw (ReadingException);
				
				  uint8_t read_u8 (cstr name) throw(ReadingException);
				   int8_t read_i8 (cstr name) throw(ReadingException);
				  int16_t read_i16(cstr name) throw(ReadingException);
				  int32_t read_i32(cstr name) throw(ReadingException);
				float32_t read_f32(cstr name) throw(ReadingException);
				float64_t read_f64(cstr name) throw(ReadingException);
				
				void read_u8   (  uint8_t**, size_t* n, cstr name) throw(ReadingException);
				void read_i8   (   int8_t**, size_t* n, cstr name) throw(ReadingException);
				void read_i16  (  int16_t**, size_t* n, cstr name) throw(ReadingException);
				void read_i32  (  int32_t**, size_t* n, cstr name) throw(ReadingException);
				void read_f32  (float32_t**, size_t* n, cstr name) throw(ReadingException);
				void read_f64  (float64_t**, size_t* n, cstr name) throw(ReadingException);
				
				void readBytes(void**, size_t* n, cstr name) throw(ReadingException);
				std::string readString(cstr name) throw(ReadingException);
				
				void readObject(Readable*r, cstr name="") throw(ReadingException);
				Readable* readObject(cstr name="") throw(ReadingException);
				
			void doneReading() throw(ReadingException);
	
			
			void error(cstr message) throw(ReadingException) { throwIt(message); }
	
		///
		/// Private stuff
		///
			private:
				std::istream * is;
				struct Context {
					enum State {ReadingBuffer, ReadingObject, ReadingName}; State state;
	
					int baseoffset;
					int index, length;
					// a negative length means no limit
					
					string name_readName;
					string ro_className;
					unsigned char ro_version;
					
					bool ro_done_startReading;
					bool ro_done_doneReading;
					
					
					Context(State state, int baseoffset, int length)
					: state(state), baseoffset(baseoffset), index(0),length(length) {
						ro_done_startReading = false;
						ro_done_doneReading = false;
					}
					
					std::string describe();
				};

				std::vector<Context> stack;
				
				/** Reference to last context in the stack. */
				Context & context() throw(ReadingException);
				
				void createContext(Context::State state,int length)throw(ReadingException);
				void enterContext(Context::State)throw(ReadingException);
				
				void exitContext() throw(ReadingException);
				void advance(int n) throw(ReadingException);
				void expectState(Context::State state) throw(ReadingException);
				/** Returns a description of the stack. */
				std::string printStackTrace();
			
			
				void assertValidRead(cstr function) throw(ReadingException);
				bool checkName(cstr name) throw(ReadingException);
	
				/** Throws an exception, writing also the stack trace. */
				void throwIt(std::string error) throw (ReadingException);
			
					
				void checkInputStream() throw (ReadingException);
				
				int8_t    stream_peek_int8()    throw (ReadingException);
				
				int8_t    stream_read_int8()    throw (ReadingException);
				int16_t   stream_read_int16()   throw (ReadingException);
				int32_t   stream_read_int32()   throw (ReadingException);
				float32_t stream_read_float32() throw (ReadingException);
				float64_t stream_read_float64() throw (ReadingException);
				
				int8_t    real_peek_int8()      throw (ReadingException);
				
				int8_t    real_read_int8()      throw (ReadingException);
				int16_t   real_read_int16()     throw (ReadingException);
				int32_t   real_read_int32()     throw (ReadingException);
				float32_t real_read_float32()   throw (ReadingException);
				float64_t real_read_float64()  throw (ReadingException);
		
				std::string stream_read_string() throw(ReadingException);
				void expectPacket(char expected, std::string message) throw(ReadingException);
	};
	
	}}} // namespace

#endif
