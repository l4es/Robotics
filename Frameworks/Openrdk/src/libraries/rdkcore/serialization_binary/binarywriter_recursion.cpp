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
#include "binarywriter.h"

namespace RDK2 { namespace Serialization { namespace Binary {
			
		BinaryWriter::Context & BinaryWriter::context() throw(WritingException) {
			if(!stack.size()) throwIt("Empty stack.");
			return (stack.back());
		}
		
		void BinaryWriter::startWriting(cstr myClassName, unsigned char version)throw(WritingException) {
			Context c; 
			c.state = Context::StartWriting;
			c.sw_className=myClassName;
			c.sw_version = version;
			stack.push_back(c);
		}
		
		void BinaryWriter::doneWriting()throw(WritingException) {
			if(context().state != Context::StartWriting) {
				throwIt("doneWriting(): called not in context StartWriting.");	
			}
			std::string objectClassName = context().sw_className;
			Buffer objectPayload = context().payload;
			unsigned char version = context().sw_version;
			stack.pop_back();
			
			if(context().state == Context::WriteObject) {
				context().wo_className = objectClassName;
				context().wo_version = version;
				context().payload = objectPayload;
			} else throwIt("doneWriting(): expected previous context to be WriteObject");
		}
		
		void BinaryWriter::throwIt(std::string error) throw (WritingException) {
				throw WritingException(error+'\n'+printStackTrace());	
		}
		
		void BinaryWriter::appendBuffer(const Buffer& b) throw(WritingException) {
			if(!stack.size()) throwIt("Empty stack.");
			
			(stack.back()).payload.append(b);
		}
		
		
		void BinaryWriter::writeObject( bool needClassName, const Writable*w, cstr name) 
			throw(WritingException) {
			Context c; 
			c.state = Context::WriteObject;
			stack.push_back(c);
					w->write(this);
			Buffer className = context().wo_className;
			unsigned char version = context().wo_version;
			Buffer payload = context().payload;
			stack.pop_back();
					
			if(needClassName)
			appendBuffer( createNamePacket( name, packet_objectWithClass(className, version, payload)));
			else
			appendBuffer( createNamePacket( name, packet_objectWithoutClass(payload)));
		}
		
		Buffer BinaryWriter::serialize(bool withClassName, const Writable*w) {
			Context c; 
			c.state = Context::Serialize;
			stack.push_back(c);
				writeObject(withClassName, w);
			Buffer b = context().payload;
			stack.pop_back();
			return b;
		}

		std::string BinaryWriter::printStackTrace() {
			ostringstream oss;
			oss << "Stack size: " << stack.size() << endl;
			for (int i=0;i<(int)stack.size();i++) {
				oss << "\t " << i << ". " << stack[i].describe() << std::endl;	
			}
			return oss.str();
		}
		
		std::string BinaryWriter::Context::describe() {
			switch(state) {
				case(WriteObject): 
					return string("WriteObject, className=")+wo_className;
				
				case(StartWriting):
					return string("StartWriting, className=")+sw_className;

				case(Serialize):
					return "Serialize";

				case(ObjectArray):
					return "ObjectArray, current objects = XXX";

				default: 
					return "Unknown state.";
			}
		}
		
	}}} // end namespace 
