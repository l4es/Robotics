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

#ifndef RDK2_TEXTUTILS_LINESTREAM
#define RDK2_TEXTUTILS_LINESTREAM

#include <iostream>

#include "textutils.h"

namespace RDK2 { namespace TextUtils {


	/**
	 * This class allows to read a text file, line by line, 
	 * without seeing lines beginnings with a comment character (ex: "#").
	 *
	 */
	class LineStream {
		public:
			LineStream(std::istream&i, char commentChar = '#'); 
			LineStream(std::istream*i, char commentChar = '#');
		
			/** Returns false if stream fail()s. */
			bool readLine();
			
			/** Returns last line read. */
			const std::string& getLine() { return line; }
			
			/** Returns last comment read. */
			const std::string& getLastComment() { return lastComment; }
			
			/** Returns a istream to a istringstream of last line. */
			//std::istream& getLineStream();
			
			/** Tokenizes last line */
			StringVector tokenize(); 

		private:
			std::istream *is;
			std::string lastComment;
			std::string line;
			std::istringstream linestream;
			char commentChar;
	};

}} // ns

#endif
