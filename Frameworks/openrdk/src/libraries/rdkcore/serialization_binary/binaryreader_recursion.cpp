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

#include <sstream>

#include <rdkcore/object/objectmanager.h>

#include "binaryreader.h"
#include "constants.h"
#include "packets.h"


namespace RDK2 { namespace Serialization { namespace Binary {

	BinaryReader::BinaryReader() {}
	BinaryReader::~BinaryReader(){}
	
			
	BinaryReader::Context & BinaryReader::context() throw(ReadingException) {
		if(!stack.size())
			throwIt("Empty stack");
		return stack.back();
	}
				
	void BinaryReader::createContext(Context::State state, int length) throw(ReadingException) {
		stack.push_back(Context(state, 0, length));
	}
	
	void BinaryReader::enterContext(Context::State state) throw(ReadingException) {
		// check before
		int length = stream_read_int32();
		BINARY_READER_DEBUG("entering context of length " << length);
		size_t offset = context().baseoffset + context().index;
		advance(length);
		stack.push_back(Context(state, offset , length));
	}
	
	void BinaryReader::advance(int n) throw(ReadingException) {
		context().index = (size_t) (context().index + n);
		// negative means no limit
		if(context().length>=0)
		if(context().index>context().length) {
			throwIt("Gone past available data"); 
		}
	}
	
	void BinaryReader::expectState(Context::State state) throw(ReadingException) {
		if(context().state != state) {
			throwIt(string("I expected to be in state ") /*+ toString(state) XXX */ );	
		}
	}

	void BinaryReader::exitContext() throw(ReadingException) {
/*		if(context().index<context().length) {
			throwIt("There are bytes left.");
		}*/	//FIXME prenditela con giorgio
		
		BINARY_READER_DEBUG("exiting context of length " << context().length);
		stack.pop_back();
	}
	
	void  BinaryReader::assertValidRead(cstr function) throw(ReadingException) {
		expectState(Context::ReadingObject);
		if(!context().ro_done_startReading) {
			throwIt(function + " called before startReading() ");
		}
	}
		
	Readable* BinaryReader::readObject(cstr name) throw(ReadingException) {
		bool present = checkName(name);
		
		int8_t type = stream_read_int8();
		if(type!=PACKET_HEADER_OBJECTWITHCLASS) 
			throwIt("I need a class name to instantiate");
			
			enterContext(Context::ReadingObject);
				context().ro_version = stream_read_int8();
				std::string className = context().ro_className = stream_read_string();
			
				// instantiate
				Readable * r = RDK2::Meta::forName(context().ro_className); 
				if(!r)
					throwIt(
						string("Could not instantiate object of class")
						+context().ro_className); 	
				
				r->read(this);
				
				if(!context().ro_done_startReading)
					throwIt(string("Object of class ") + className 
						+ " did not call startReading().");	
				
				if(!context().ro_done_doneReading)
					throwIt(string("Object of class ") + className 
						+ " did not call doneReading().");	
				
			exitContext();
		if(present) exitContext();
		return r;
	}
		
	void BinaryReader::readObject(Readable*r, cstr name) throw(ReadingException) {
		BINARY_READER_DEBUG("Reading object, name = "<<name);
		bool present = checkName(name);
		BINARY_READER_DEBUG("Name present: "<<present);
		
		int8_t type = stream_read_int8();
		BINARY_READER_DEBUG("Packet is: "<<packetName(type));
			
			switch(type) {
				case(PACKET_HEADER_OBJECTWITHCLASS): {
					enterContext(Context::ReadingObject);
						context().ro_version = stream_read_int8();
						context().ro_className = stream_read_string();
						BINARY_READER_DEBUG("Class name is: "<<context().ro_className);
					break;
				}
				case(PACKET_HEADER_OBJECTNOCLASS):{
					enterContext(Context::ReadingObject);
						context().ro_className = "";
						context().ro_version = 1;
					break;
				}
				default:
					throwIt("It is neither a name or object packet");
			}
			
					r->read(this);
			
			string className = context().ro_className;

			if(!context().ro_done_startReading)
				throwIt(string("Object of class ") + className + " did not call startReading().");	
			
			if(!context().ro_done_doneReading)
				throwIt(string("Object of class ") + className + " did not call doneReading().");	
						
		exitContext();
		if(present) exitContext();
	}
	
	unsigned char BinaryReader::startReading(cstr className) throw (ReadingException) {
		expectState(Context::ReadingObject);
		
		if(context().ro_done_startReading) {
			throwIt(string("startReading() already called (class=")+className+")"); 	
		}
		
		context().ro_done_startReading = true;
		
		if( (context().ro_className != "") && (context().ro_className != className)) {
			throwIt(string("I expected class ")+className+", instead I read "+context().ro_className);
		}
		
		if( context().ro_className == "") {
			context().ro_className = string(" (declared by Readable: ")+className+") "; 
		}
		BINARY_READER_DEBUG( "started reading class " << className);
		return context().ro_version;
	}
	
	void BinaryReader::doneReading() throw (ReadingException) {
		expectState(Context::ReadingObject);
		if(!context().ro_done_startReading) {
			throwIt("doneReading() called before startReading()."); 	
		}
		if(context().ro_done_doneReading) {
			throwIt("doneReading() already called."); 	
		}
		context().ro_done_doneReading = true;
	}
	
		
	Readable * BinaryReader::deserialize(cstr buffer) {
		BINARY_READER_DEBUG(endl<<endl<<endl<<"Started reading from buffer" << endl);
		istringstream is(buffer);
		this->is = &is;
		stack.clear();
		createContext(Context::ReadingBuffer, buffer.length());
			Readable *r = readObject();
		exitContext();
		return r;
	}
	
	void BinaryReader::deserialize(cstr buffer, Readable *r) {
		BINARY_READER_DEBUG(endl<<endl<<endl<<"Started reading from buffer" << endl);
		istringstream is(buffer);
		this->is = &is;
		stack.clear();
		createContext(Context::ReadingBuffer, buffer.length());
			readObject(r);
		exitContext();
	}

	}}} // end namespace RDK2
