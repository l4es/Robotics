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

#include "linestream.h"
#include "textutils.h"

namespace RDK2 { namespace TextUtils {

	using namespace std;
	
	LineStream::LineStream(std::istream&i, char commentChar) : 
		is(&i), commentChar(commentChar) {}
	
	LineStream::LineStream(std::istream*i, char commentChar) : 
		is(i),  commentChar(commentChar) {}
		
	bool LineStream::readLine() {
		lastComment = string("");
	
		while(1) {
			std::getline(*is,line);
	
			if(is->fail()) {
				return false;
			}

			// We ignore empty lines
			if(line.empty()) {
				continue;
			}
			
			// If line is a comment
			if(line[0]==commentChar) {
				// We save the rest of the comment line 
				lastComment += line.substr(1).append("\n");
				// and we go forward in the file
				continue;
			} else {
				return true;	
			}
		} 
	}
	
	vector<string> LineStream::tokenize() {
		return RDK2::TextUtils::tokenize(getLine());
	}
	
	/*
	std::istream& LineStream::getLineStream() { 
		linestream.str(line);
		cerr << "Reading now line " << line << endl;
		return linestream; 
	}*/
		
	
}} // ns

