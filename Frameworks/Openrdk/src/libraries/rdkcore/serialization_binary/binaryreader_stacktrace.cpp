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
#include "binaryreader.h"
#include "assert.h"

namespace RDK2 { namespace Serialization { namespace Binary {
			
	void BinaryReader::throwIt(std::string error) throw (ReadingException) {
		if(BINARY_READER_ABORT_ON_EXCEPTION) {
			cerr << error<< endl << printStackTrace() << endl;
			assert(false);
		} else
			throw ReadingException(error+'\n'+printStackTrace());	
	}
	
	std::string BinaryReader::printStackTrace() {
		ostringstream oss;
		oss << "Stack size: " << stack.size() << endl;
		for (int i=0;i<(int)stack.size();i++) {
			oss << "\t " << i << ". " << stack[i].describe() << std::endl;	
		}
		return oss.str();
	}
	
	std::string BinaryReader::Context::describe() {
		ostringstream os;
		switch(state) {
			case(ReadingBuffer): {
				os << " ReadingBuffer ";
			} break;
			case(ReadingName): {
				os << " Reading named object: " << name_readName;
			} break;
			case(ReadingObject): {
				os << " Reading an object, className: " << ro_className;
			} break;
			default: 
				os << " Unknown state (!) ";
		}
		
		os << " segment of length " << length << " ("<<index<<" cnsmd) at address "<<baseoffset;
		return os.str();;
	}

	}}} // end namespace
