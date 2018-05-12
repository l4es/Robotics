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

#include <math.h>
#include <rdkcore/serialization/read.h>
#include <rdkcore/serialization/write.h>
#include <rdkcore/textutils/textutils.h>
#include <rdkcore/object/object.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/geometry/utils.h>
#include <rdkcore/ns.h>

#define LOGGING_MODULE "GoodTests"

using namespace RDK2;

namespace RDK2 {
	namespace UnitTest {
		using namespace std;
		
		
		struct EmptyClass: public RDK2::Object {
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("EmptyClass");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("EmptyClass");
				r->doneReading();
			}
			
			Object * clone() const { return new EmptyClass(); }

			std::vector<Object*> getTestCases() const { 
				vector<Object*> v;
				v.push_back(new EmptyClass());
				return v; 
			}

			bool equals(const Object*) const { return true; };
		};
		
		RDK2_FACTORY(EmptyClass);
	
	
	
		struct IntWrapper: public RDK2::Object {
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

			Object * clone() const { return new IntWrapper(i); }

			std::vector<Object*> getTestCases() const { 
				vector<Object*> v;
				v.push_back(new IntWrapper(-1));
				v.push_back(new IntWrapper(1));
				return v; 
			}

			bool equals(const Object*ob) const { 
				const IntWrapper * o = dynamic_cast<const IntWrapper*>(ob);
				return o && o->i == i; 
			}
		};
		
		RDK2_FACTORY(IntWrapper);
		
		struct Complex1:  public RDK2::Object {
			IntWrapper s;
			Complex1(int i=-1) : s(i) {}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("Complex1");
					w->writeObject(false, &s, "s");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("Complex1");
					r->readObject( &s, "s");
				r->doneReading();
			}

			Object * clone() const { return new Complex1(s.i); }

			std::vector<Object*> getTestCases() const { 
				vector<Object*> v;
				v.push_back(new Complex1(42));
				return v; 
			}

			bool equals(const Object*ob) const { 
				const Complex1 * c = dynamic_cast<const Complex1*>(ob);
				return c && c->s.equals(&s);
			}
		};
		RDK2_FACTORY(Complex1);
		
		struct Complex2: public RDK2::Object {
			IntWrapper s;
			Complex2(int i=-1) : s(i) {}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("Complex2");
					w->writeObject(true, &s, "s");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("Complex2");
					r->readObject( &s, "s");
				r->doneReading();
			}

			Object * clone() const { return new Complex2(s.i); }

			std::vector<Object*> getTestCases() const { 
				vector<Object*> v;
				v.push_back(new Complex1(42));
				return v; 
			}

			bool equals(const Object*ob) const { 
				const Complex2 * c = dynamic_cast<const Complex2*>(ob);
				return c && c->s.equals(&s);
			}
		};
		RDK2_FACTORY(Complex2);
		
		
		struct Complex6: public RDK2::Object {
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
			
			Object * clone() const { return new Complex6(x,y); }
			
			
			std::vector<Object*> getTestCases() const { 
				vector<Object*> v;
				v.push_back(new Complex6(42,32));
				return v; 
			}

			bool equals(const Object*ob) const { 
				const Complex6 * c = dynamic_cast<const Complex6*>(ob);
				return c && (c->x == x) && (c->y == y);
			}
		};
		RDK2_FACTORY(Complex6);
		
		
		struct Complex7: public RDK2::Object {
			int x,y;
			Complex7(int x=1,int y=2): x(x), y(y) {}
			
			Object * clone() const { return new Complex7(x,y); }
			
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
			
			bool equals(const Object*ob) const { 
				const Complex7 * c = dynamic_cast<const Complex7*>(ob);
				if(!c) return false;
				return x == c->x && y == c->y;
			}
		};
		RDK2_FACTORY(Complex7);
		
	
	
		struct Complex3: public RDK2::Object {
			IntWrapper a,b; EmptyClass s;
			Complex3(int a=-1,int b=-1):a(a),b(b) {}
			
			Object * clone() const { return new Complex3(a.i,b.i); }

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

			bool equals(const Object*ob) const { 
				const Complex3 * c = dynamic_cast<const Complex3*>(ob);
				if(!c) return false;
				return a.equals(&(c->a)) && b.equals(&(c->b));
			}
		};
		RDK2_FACTORY(Complex3);
		

		struct String1: public RDK2::Object {
			std::string s;
			String1(cstr s = "") : s(s) { }

			Object * clone() const { return new String1(s); }

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
			
			bool equals(const Object*ob) const { 
				const String1 * c = dynamic_cast<const String1*>(ob);
				if(!c) return false;
				return s == c->s;
			}
		};
		RDK2_FACTORY(String1);
		

		struct Data1: public RDK2::Object {
			char* buffer; size_t size; int seed;
			Data1(size_t size=5, int seed=100) : size(size), seed(seed) {
				buffer = new char[size];
				for (size_t i = 0; i < size; i++) 
					buffer[i] = i % seed;
			}

			~Data1() { delete[] buffer; }

			void write(Writer* w) const throw (WritingException) {
				w->startWriting("Data1");
					w->writeBytes(buffer, size, "bytes");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				delete buffer;
				r->startReading("Data1");
					r->readBytes((void**) &buffer, &size, "bytes");
				r->doneReading();
			}
			
			Object * clone() const { 
				return new Data1(size, seed);
			}
			
			std::vector<Object*> getTestCases() const { 
				vector<Object*> v;
				v.push_back(new Data1(10, 4));
				v.push_back(new Data1(100, 15));
				return v; 
			}

			bool equals(const Object*ob) const { 
				const Data1 * c = dynamic_cast<const Data1*>(ob);
				if(!c) return false;
				if(size != c->size) {
					// RDK_ERROR_PRINTF("Different size: %d instead of %d.", c->size, size);
					return false;
				}
				for(size_t i=0;i<size;i++)
					if(buffer[i]!=c->buffer[i]) {
					//	RDK_ERROR_PRINTF("Different byte %d/%d: %d instead of %d.",
					//	 	i, size,c-> buffer[i],buffer[i]);
						return false;
					}
				return true;
			}
		};
		RDK2_FACTORY(Data1);
		

		struct StringAndData1: public RDK2::Object {
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

			Object * clone() const { return new StringAndData1(s1.s); }
			
			bool equals(const Object*ob) const { 
				const StringAndData1 * c = dynamic_cast<const StringAndData1*>(ob);
				if(!c) return false;
				return s1.equals(&(c->s1));
			}
		};
		RDK2_FACTORY(StringAndData1);
		
		template<class T>
		struct ArrayInt: public RDK2::Object {
			T* buffer; size_t size; 
			int seed;
			string myClassName;
			
			ArrayInt(size_t size, int seed, string className) : size(size), seed(seed)
			{
				buffer = new T[size];
				for(size_t i = 0; i < size; i++)
				 buffer[i] = (T) ( i % seed - seed/2 + 0.3);
				
				myClassName = className;
			}
			
			~ArrayInt() { delete[] buffer; }
			

			bool equals(const Object*ob) const { 
				const ArrayInt<T> * c = dynamic_cast<const ArrayInt<T>*>(ob);
				if(!c) return false;
				if(size != c->size) {
					 // RDK_ERROR_PRINTF("%s: Different size: %d instead of %d.", myClassName.c_str(), c->size, size);
					return false;
				}
				for(size_t i=0;i<size;i++)
					if(buffer[i]!=c->buffer[i]) {
						//RDK_ERROR_PRINTF("%s: Different byte %d/%d: %d instead of %d.", myClassName.c_str(),
						// 	i, size,c-> buffer[i],buffer[i]);
						return false;
					}
				return true;
			}
			
			string getClassName() const { return myClassName; }
		};

		template<class T>
		struct ArrayFloat: public ArrayInt<T> {
			ArrayFloat(size_t size, int seed, string className):
				ArrayInt<T>(size,seed,className){}
			
			bool equals(const Object*ob) const { 
				const ArrayInt<T> * c = dynamic_cast<const ArrayInt<T>*>(ob);
				if(!c) return false;
				if(this->size != c->size) {
					 // RDK_ERROR_PRINTF("%s: Different size: %d instead of %d.", myClassName.c_str(), c->size, size);
					return false;
				}
				for(size_t i=0;i<this->size;i++)
					if(!sameFloat(this->buffer[i],c->buffer[i], (T)1e-10))
						return false;
					
				return true;
			}
			
		};


		template<class CLA>
		std::vector<Object*> getStandardTestCases() { 
			vector<Object*> v;
			v.push_back(new CLA(0, 12));
			v.push_back(new CLA(10, 1));
			v.push_back(new CLA(10, 4));
			v.push_back(new CLA(100, 15));
			return v; 
		}

		struct ArrayUint8: public ArrayInt<uint8_t> {
			ArrayUint8(size_t size=1, int seed=3) : 
				ArrayInt<uint8_t>(size,seed,"ArrayUint8") {  }

			std::vector<Object*> getTestCases() const { 
				return getStandardTestCases<ArrayUint8>(); }

			Object * clone() const { 
				return new ArrayUint8(size, seed);
			}

			void write(Writer* w) const throw (WritingException) {
				w->startWriting(myClassName);
					w->write_u8(buffer, size, "buffer");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				delete[] buffer;
				r->startReading(myClassName);
					r->read_u8(&buffer, &size, "buffer");
				r->doneReading();
			}
		};

		RDK2_FACTORY(ArrayUint8);

		struct ArrayInt8: public ArrayInt<int8_t> {
			ArrayInt8(size_t size=1, int seed=3) : 
				ArrayInt<int8_t>(size,seed,"ArrayInt8") {}

			Object * clone() const { 
				return new ArrayInt8(size, seed);
			}

			std::vector<Object*> getTestCases() const { 
				return getStandardTestCases<ArrayInt8>(); }
	
			void write(Writer* w) const throw (WritingException) {
				w->startWriting(myClassName);
					w->write_i8(buffer, size, "buffer");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				delete[] buffer;
				r->startReading(myClassName);
					r->read_i8(&buffer, &size, "buffer");
				r->doneReading();
			}
		};

		RDK2_FACTORY(ArrayInt8);
		
		struct ArrayInt32: public ArrayInt<int32_t> {
			ArrayInt32(size_t size=1, int seed=3) : 
				ArrayInt<int32_t>(size,seed,"ArrayInt32") { }
			
			std::vector<Object*> getTestCases() const { 
				return getStandardTestCases<ArrayInt32>(); }
		
			Object * clone() const {  return new ArrayInt32(size, seed); }			

			void write(Writer* w) const throw (WritingException) {
				w->startWriting(myClassName);
					w->write_i32(buffer, size, "buffer");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				delete[] buffer;
				r->startReading(myClassName);
					r->read_i32(&buffer, &size, "buffer");
				r->doneReading();
			}
		};

		RDK2_FACTORY(ArrayInt32);

		struct ArrayInt16: public ArrayInt<int16_t> {
			ArrayInt16(size_t size=1, int seed=3) : 
				ArrayInt<int16_t>(size,seed,"ArrayInt16") { }

			std::vector<Object*> getTestCases() const { 
				return getStandardTestCases<ArrayInt16>(); }

			Object * clone() const {  return new ArrayInt16(size, seed); }
				
			void write(Writer* w) const throw (WritingException) {
				w->startWriting(myClassName);
					w->write_i16(buffer, size, "buffer");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				delete[] buffer;
				r->startReading(myClassName);
					r->read_i16(&buffer, &size, "buffer");
				r->doneReading();
			}
		};

		RDK2_FACTORY(ArrayInt16);
		
		struct ArrayFloat32: public ArrayFloat<float32_t> {
			ArrayFloat32(size_t size=1, int seed=3) : 
				 ArrayFloat<float32_t>(size,seed,"ArrayFloat32") {  }

				std::vector<Object*> getTestCases() const { 
					std::vector<Object*> v = getStandardTestCases<ArrayFloat32>(); 

					ArrayFloat32 * af = new ArrayFloat32(3,1);
					af->buffer[0] = RDK_NAN;
					af->buffer[2] = RDK_NAN;
					v.push_back(af);

					af = new ArrayFloat32(3,1);
					af->buffer[1] = RDK_NAN;
					v.push_back(af);

					af = new ArrayFloat32(3,1);
					af->buffer[1] = RDK_INF32;
					v.push_back(af);

					return v;
				}

			Object * clone() const {  return new ArrayFloat32(size, seed); }
				
			void write(Writer* w) const throw (WritingException) {
				w->startWriting(myClassName);
					w->write_f32(buffer, size, "buffer");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				delete[] buffer;
				r->startReading(myClassName);
					r->read_f32(&buffer, &size, "buffer");
				r->doneReading();
			}
		};

		RDK2_FACTORY(ArrayFloat32);

		
		struct ArrayFloat64: public ArrayFloat<float64_t> {
			ArrayFloat64(size_t size=1, int seed=3) : 
				 ArrayFloat<float64_t>(size,seed,"ArrayFloat64") { }

			std::vector<Object*> getTestCases() const { 
				std::vector<Object*> v = getStandardTestCases<ArrayFloat64>(); 

				ArrayFloat64 * af = new ArrayFloat64(3,1);
				af->buffer[0] = RDK_NAN;
				af->buffer[2] = RDK_NAN;
				v.push_back(af);

				 af = new ArrayFloat64(3,1);
				af->buffer[1] = RDK_NAN;
				v.push_back(af);

				 af = new ArrayFloat64(3,1);
				af->buffer[1] = RDK_INF64;
				v.push_back(af);

				return v;
			}

			Object * clone() const {  return new ArrayFloat64(size, seed); }
				
			void write(Writer* w) const throw (WritingException) {
				w->startWriting(myClassName);
					w->write_f64(buffer, size, "buffer");
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				delete[] buffer;
				r->startReading(myClassName);
					r->read_f64(&buffer, &size, "buffer");
				r->doneReading();
			}
		};

		RDK2_FACTORY(ArrayFloat64);
 
		struct VersionTest: public RDK2::Object {
			unsigned char version;

			VersionTest(unsigned char v=100) : version(v) {

			}

			void write(Writer* w) const throw (WritingException) {
				w->startWriting("VersionTest", version);
					w->write_u8(version);
				w->doneWriting();
			}

			void read(Reader* r) throw (ReadingException) {
				unsigned char v1 = r->startReading("VersionTest");
					unsigned char v2 = r->read_u8();
				r->doneReading();
				if(v1 != v2) {
					throw ReadingException(string("Version number is different: wrote ")
						+toString((int)v2)+
						" but read "+toString((int)v1));
				}
				this->version = v1;
			}

			Object * clone() const { return new VersionTest(version); }
			
			std::vector<Object*> getTestCases() const { 
				vector<Object*> v;
				v.push_back(new VersionTest(10));
				v.push_back(new VersionTest(30));
				v.push_back(new VersionTest(40));
				return v; 
			}

			bool equals(const Object*ob) const { 
				const VersionTest * c = dynamic_cast<const VersionTest*>(ob);
				return c && (c->version==version);
			}
		};
		RDK2_FACTORY(VersionTest);
		
		
		struct EmptyVersion: public RDK2::Object {

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

			Object * clone() const { return new  EmptyVersion(); }
			
			std::vector<Object*> getTestCases() const { 
				vector<Object*> v;
				v.push_back(new EmptyVersion());
				return v; 
			}

			bool equals(const Object*ob) const { 
				const EmptyVersion * c = dynamic_cast<const EmptyVersion*>(ob);
				return c != 0;
			}
		};
		RDK2_FACTORY(EmptyVersion);
		
	
	}} // namespace RDK2::UnitTest
#endif
