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

#ifndef H_BAD_TESTS
#define H_BAD_TESTS

#include <rdkcore/serialization/read.h>
#include <rdkcore/serialization/write.h>
#include "good_tests.h"

namespace RDK2 { namespace UnitTest { namespace Serialization {
	
		
		struct ErrorNoStartWriting: public Writable, public Readable {
			void write(Writer*w) const throw (WritingException) {
				//w->startWriting("ErrorNoStartWriting");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("ErrorNoStartWriting");
				r->doneReading();
			}
		};
		
		struct ErrorNoDoneWriting: public Writable, public Readable {
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("ErrorNoStartWriting");
				//w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("ErrorNoStartWriting");
				r->doneReading();
			}
		};
	
		struct ErrorNoStartReading: public Writable, public Readable {
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("ErrorNoStartWriting");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				//r->startReading("ErrorNoStartWriting");
				r->doneReading();
			}
		};
		
		
		struct ErrorNoDoneReading: public Writable, public Readable {
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("ErrorNoStartWriting");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("ErrorNoStartWriting");
				//r->doneReading();
			}
		};
	
		
		struct ErrorWrongClass: public Writable, public Readable {
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("ErrorWrongClass");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("WrongClass");
				r->doneReading();
			}
		};
		
		
		struct ErrorWrongType1: public Writable, public Readable {
			int i;
			ErrorWrongType1(int i=3):i(i){}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("ErrorWrongType1");
					w->write_i32(i, "i");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("ErrorWrongType1");
					r->read_f32("i");
				r->doneReading();
			}
		};
		
		
		struct ErrorWrongName1: public Writable, public Readable {
			int i;
			ErrorWrongName1(int i=3):i(i){}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("ErrorWrongName1");
					w->write_i32(i, "i");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("ErrorWrongName1");
					i = r->read_i32("j");
				r->doneReading();
			}
		};
		
		
		struct ErrorIncompleteRead: public Writable, public Readable {
			int i;
			ErrorIncompleteRead(int i=3):i(i){}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("ErrorIncompleteRead");
					w->write_i32(i, "i");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("ErrorIncompleteRead");
				r->doneReading();
			}
		};
		
		struct ErrorTooMuchRead: public Writable, public Readable {
			int i;
			ErrorTooMuchRead():i(42){}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("ErrorTooMuchRead");
					//w->write_i32(i, "i");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("ErrorTooMuchRead");
					i = r->read_i32("i");
				r->doneReading();
			}
		};
		
		
		struct ErrorWrongMemberClass: public Writable, public Readable {
			IntWrapper a,b; EmptyClass s;
			ErrorWrongMemberClass(int a=1,int b=2):a(a),b(b) {}
			
			void write(Writer*w) const throw (WritingException) {
				w->startWriting("ErrorWrongMemberClass");
					w->writeObject(true, &a, "a");
					w->writeObject(false, &s, "s");
				w->doneWriting();
			}
			
			void read(Reader*r) throw (ReadingException) {
				r->startReading("ErrorWrongMemberClass");
					r->readObject( &s, "s");
					r->readObject( &a, "a");
				r->doneReading();
			}
		};
		
}}}

#endif
