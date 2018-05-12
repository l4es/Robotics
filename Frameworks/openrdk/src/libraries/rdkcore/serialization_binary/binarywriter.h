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

#ifndef H_BINARY_WRITER
#define H_BINARY_WRITER

#include <vector>
#include <rdkcore/serialization/write.h>

#include "packets.h"


namespace RDK2 { namespace Serialization { namespace Binary {
			
	/** A binary writer */
	class BinaryWriter: public Writer {
		
		///
		/// Public interface
		///
		public:
			BinaryWriter(bool writeNames) : writeNames(writeNames) {};
			virtual ~BinaryWriter();
			
			/** Serializes the object to a string. */
			std::string serialize(bool withClassName, const Writable*w);
		
		///
		/// Writer interface
		///
		public:
			void startWriting(cstr myClassName, unsigned char version) throw(WritingException);
				
				void write_u8(uint8_t, cstr name) throw(WritingException);
				void write_i8(int8_t, cstr name) throw(WritingException);
				void write_i16(int16_t,cstr name) throw(WritingException);
				void write_i32(int32_t, cstr name) throw(WritingException);
				void write_f32(float32_t, cstr name)throw(WritingException);
				void write_f64(float64_t, cstr name)throw(WritingException);
				
				void write_u8(const uint8_t*, size_t n, cstr name)throw(WritingException);
				void write_i8(const int8_t*, size_t n, cstr name)throw(WritingException);
				void write_i16(const int16_t*, size_t n, cstr name)throw(WritingException);
				void write_i32(const int32_t*, size_t n, cstr name)throw(WritingException);
				void write_f32(const float32_t*, size_t n, cstr name)throw(WritingException);
				void write_f64(const float64_t*, size_t n, cstr name) throw(WritingException);
				void writeString(cstr s, cstr name) throw(WritingException) ;
				
				void writeBytes(const void* i, size_t n, cstr name) throw(WritingException) ;

				void writeObject( bool needClassName, const Writable*w,cstr name="") 
					throw(WritingException);
			
			void doneWriting() throw(WritingException);
		
		private:
			/** Encloses payload in a name packet */
			Buffer createNamePacket(cstr name, Buffer payload);
			/** If false, names will be omitted */
			bool writeNames;	
			
			///
			/// Structures and functions to aid with recursion
			///
			struct Context {
				enum { WriteObject, StartWriting, Serialize, ObjectArray } state;
				Buffer payload;
				
				Buffer wo_className; ///< usato da WriteObject
				unsigned char wo_version;///< usato da StartWriting
				
				Buffer sw_className; ///< usato da StartWriting
				unsigned char sw_version;///< usato da StartWriting
				
				std::string describe();
			};
			std::vector<Context> stack;
			/** Reference to last context in the stack. */
			Context & context() throw(WritingException);
			/** Returns a description of the stack. */
			std::string printStackTrace();
			
			/** Throws an exception, writing also the stack trace. */
			void throwIt(std::string error) throw (WritingException);
			
			/** Appends string to current payload */
			void appendBuffer(const Buffer&) throw(WritingException);
	};

	}}} // namespace 

#endif
