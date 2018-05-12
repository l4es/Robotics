/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
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

#ifndef H_GOOD_TESTS
#define H_GOOD_TESTS

#include <rdkcore/serialization/read.h>
#include <rdkcore/serialization/write.h>
#include <rdkcore/textutils/textutils.h>

using namespace RDK2::TextUtils;
using namespace RDK2::Serialization;

namespace RDK2 {
	namespace UnitTest {
	namespace Serialization {
	
		struct EmptyClass: public Writable, public Readable {
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("EmptyClass");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("EmptyClass");
				r->doneReading();
			}
		};
	
		struct IntWrapper: public Writable, public Readable {
			int i;
			IntWrapper(int i):i(i){}
			IntWrapper():i(-3){}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("IntWrapper");
					w->write_i32(i, "i");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("IntWrapper");
					i = r->read_i32("i");
				r->doneReading();
			}
		};
		
		struct Complex1: public Writable, public Readable {
			EmptyClass s;
			Complex1() {}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("Complex");
					w->writeObject(false, &s, "s");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("Complex");
					r->readObject( &s, "s");
				r->doneReading();
			}
		};
		
		struct Complex2: public Writable, public Readable {
			EmptyClass s;
			Complex2() {}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("Complex");
					w->writeObject(true, &s, "s");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("Complex");
					r->readObject( &s, "s");
				r->doneReading();
			}
		};
	
		struct Complex4: public Writable, public Readable {
			IntWrapper a;
			Complex4():a(2) {}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("Complex");
					w->writeObject(false, &a, "a");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("Complex");
					r->readObject( &a, "a");
				r->doneReading();
			}
		};
		
		
		struct Complex6: public Writable, public Readable {
			int x,y;
			Complex6(int x=1,int y=2): x(x), y(y) {}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("Complex6");
					w->write_i32(x, "x");
					w->write_i32(y, "y");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("Complex6");
					x = r->read_i32("x");
					y = r->read_i32("y");
				r->doneReading();
			}
		};
		
		
		struct Complex7: public Writable, public Readable {
			int x,y;
			Complex7(int x=1,int y=2): x(x), y(y) {}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("Complex7");
					w->write_i32(x);
					w->write_i32(y);
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("Complex7");
					x = r->read_i32();
					y = r->read_i32();
				r->doneReading();
			}
		};
	
		struct Complex5: public Writable, public Readable {
			IntWrapper a;
			Complex5():a(2) {}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("Complex5");
					w->writeObject(false, &a, "a");
					w->write_i32(13, "mah");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("Complex5");
					r->readObject( &a, "a");
					r->read_i32("mah");
				r->doneReading();
			}
		};
		
	
		struct Complex3: public Writable, public Readable {
			IntWrapper a,b; EmptyClass s;
			Complex3(int a=-1,int b=-1):a(a),b(b) {}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("Complex3");
					w->writeObject(false, &a, "a");
					w->writeObject(false,  &b, "b");
					w->writeObject(false, &s, "s");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("Complex3");
					r->readObject( &a, "a");
					r->readObject( &b, "b");
					r->readObject( &s, "s");
				r->doneReading();
			}
		};

		struct String1: public Writable, public Readable {
			std::string s;
			String1(cstr s = "") : s(s) { }

			void write(Writer* w) const throw (WritingException) {
				w->startWriting("String1");
					w->writeString(s, "myString");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				r->startReading("String1");
					s = r->readString("myString");
				r->doneReading();
			}
		};

		#define DATA1_BUFFER_SIZE 512
		struct Data1: public Writable, public Readable {
			char* buffer;
			Data1() {
				buffer = new char[DATA1_BUFFER_SIZE];
				for (int i = 0; i < DATA1_BUFFER_SIZE; i++) buffer[i] = i % 100;
			}

			~Data1() { delete[] buffer; }

			void write(Writer* w) const throw (WritingException) {
				w->startWriting("Data1");
					w->writeBytes(buffer, DATA1_BUFFER_SIZE, "bytes");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				r->startReading("Data1");
					size_t n;
					r->readBytes((void**) &buffer, &n, "bytes");
				r->doneReading();
			}
		};

		struct StringAndData1: public Writable, public Readable {
			String1 s1;
			Data1 d1;
			StringAndData1(cstr s = "") : s1(s) { }

			void write(Writer* w) const throw (WritingException) {
				w->startWriting("StringAndData1");
					w->writeObject(false, &s1, "TheString");
					w->writeObject(true, &d1, "TheData");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				r->startReading("StringAndData1");
					r->readObject(&s1, "TheString");
					r->readObject(&d1, "TheData");
				r->doneReading();
			}
		};

		#define ARRAY_SIZE 20
		struct ArrayUint8: public Writable, public Readable {
			uint8_t* a;

			ArrayUint8() {
				a = new uint8_t[ARRAY_SIZE];
				for (int i = 0; i < ARRAY_SIZE; i++) a[i] = i % 4;
			}
			~ArrayUint8() { delete[] a; }

			void write(Writer* w) const throw (WritingException) {
				w->startWriting("ArrayUint8");
					w->write_u8(a, ARRAY_SIZE, "theArray");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				size_t sz;
				delete[] a;
				r->startReading("ArrayUint8");
					r->read_u8(&a, &sz, "theArray");
				r->doneReading();
			}
		};

		struct ArrayInt8: public Writable, public Readable {
			int8_t* a;

			ArrayInt8() {
				a = new int8_t[ARRAY_SIZE];
				for (int i = 0; i < ARRAY_SIZE; i++) a[i] = (i % 4 - 2) * 15;
			}
			~ArrayInt8() { delete[] a; }

			void write(Writer* w) const throw (WritingException) {
				w->startWriting("ArrayInt8");
					w->write_i8(a, ARRAY_SIZE, "theArray");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				size_t sz;
				delete[] a;
				r->startReading("ArrayInt8");
					r->read_i8(&a, &sz, "theArray");
				r->doneReading();
			}
		};

		struct ArrayInt32: public Writable, public Readable {
			int32_t* a;

			ArrayInt32() {
				a = new int32_t[ARRAY_SIZE];
				for (int i = 0; i < ARRAY_SIZE; i++) a[i] = (i % 4 - 2) * 15;
			}
			~ArrayInt32() { delete[] a; }

			void write(Writer* w) const throw (WritingException) {
				w->startWriting("ArrayInt32");
					w->write_i32(a, ARRAY_SIZE, "theArray");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				size_t sz;
				delete[] a;
				r->startReading("ArrayInt32");
					r->read_i32(&a, &sz, "theArray");
				r->doneReading();
			}
		};

		struct EmptyArray: public Writable, public Readable {
			int32_t* a; int n;

			EmptyArray() {
				n = 0;
				a = new int32_t[n];
			}
			~EmptyArray() { delete[] a; }

			void write(Writer* w) const throw (WritingException) {
				w->startWriting("EmptyArray");
					w->write_i32(a, n, "theArray");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				size_t sz;
				delete[] a;
				r->startReading("EmptyArray");
					r->read_i32(&a, &sz, "theArray");
				r->doneReading();
			}
		};

		struct ArrayFloat32: public Writable, public Readable {
			float32_t* a;

			ArrayFloat32() {
				a = new float32_t[ARRAY_SIZE];
				for (int i = 0; i < ARRAY_SIZE; i++) a[i] = i / 3.;
			}
			~ArrayFloat32() { delete[] a; }

			void write(Writer* w) const throw (WritingException) {
				w->startWriting("ArrayFloat32");
					w->write_f32(a, ARRAY_SIZE, "theArray");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				size_t sz;
				delete[] a;
				r->startReading("ArrayFloat32");
					r->read_f32(&a, &sz, "theArray");
				r->doneReading();
			}
		};
		
		struct VersionTest: public Writable, public Readable {
			unsigned char version;

			VersionTest() {
				this->version = 42;
			}

			void write(Writer* w) const throw (WritingException) {
				w->startWriting("VersionTest", version);
					w->write_i32(13);				
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				unsigned char version = r->startReading("VersionTest");
				r->read_i32();
				
				r->doneReading();
				if(version != this->version) {
					throw ReadingException(string("Version number is different: wrote ")+toString((int)this->version)+
						" but read "+toString((int)version));
				}
			}
		};
		
		struct EmptyVersion: public Writable, public Readable {

			void write(Writer* w) const throw (WritingException) {
				w->startWriting("EmptyVersion");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				unsigned char version = r->startReading("EmptyVersion");
				r->doneReading();
				
				if(version != 1) {
					throw ReadingException(string("Version number should be 1 instead of  ")+
						toString((int)version));
				}
			}
		};
	
	}}} // namespace RDK2::UnitTest::Serialization
#endif

