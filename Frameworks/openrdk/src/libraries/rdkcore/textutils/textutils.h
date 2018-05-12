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

#ifndef RDK2_TEXTUTILS
#define RDK2_TEXTUTILS

#include <vector>
#include <string>
#include <sstream>

namespace RDK2 { namespace TextUtils {
	
	///
	/// Types definition
	///
	
		typedef std::vector<std::string> StringVector;
		typedef const std::string& cstr;
	
	///
	/// Add your own utils here.
	///
	
		/** Consumes whitespaces from the stream. Whitespaces are empty lines, 
		isspace() and lines beginning with '#' */
		void consumeWhitespace(std::istream&is);
	//	void consume_line(std::istream&is);
	
		/** Test whether s starts with cs. */
		bool startsWith(const std::string&s, const char*cs);
		
		/** Test whether s ands with cs. */
		bool endsWith(const std::string&s, const char*cs);
		
		/** Tokenizes a string */
		StringVector tokenize(const std::string&);
		
		/** Tokenizes a string using @delimiters as delimiters **/
		std::vector<std::string> tokenize(const std::string& s, const std::string& delimiters);

		/** Reads a comment of the form "# ... \n # ..." and strips the # */
		std::string readCommentLines(std::istream&is, char commentChar='#');

		/** Reads the entire file into a string. */
   		std::string readEntireFile(cstr file);
     
	///
	/// Useful template function for parsing using << and >>.
	///
		
		/** Converts an object to string using its << operator */
		template<typename T>
		std::string toString(T v) {
			std::ostringstream oss; 
			oss << v; 
			return oss.str();
		}
	
		/** The converse of @toString. */
		template <class X>
		bool parse(const std::string&s, X&x) {
			std::istringstream iss(s);
			return iss >> x;
		}
		
		/** The converse of @toString. */
		template <class X>
		std::string quote(X&x) {
			std::string q("\"");
			return q+toString(x)+q;
		}

	
	std::string trim(const std::string& s, const char* charsToTrim = " \t");

	// non sono sicuro che questo sia il posto giusto
	std::string normalizeLibraryName(cstr libraryName);

}} // ns
	
#endif
