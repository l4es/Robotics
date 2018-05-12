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

#include <string>
#include <cstring>
#include <fstream>
#include <sys/types.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "textutils"
#include "textutils.h"
#include "linestream.h"

using namespace std;

namespace RDK2 { namespace TextUtils {
		
	StringVector tokenize(const std::string&s) {
		istringstream iss(s);
		vector<string> v;
		string t;
		while(iss>>t) { v.push_back(t); }
		return v;
	}

	vector<string> tokenize(const std::string& s, const std::string& delimiters)
	{
		vector<string> r;
		string c;
		for (size_t i = 0; i < s.size(); i++) {
			if (delimiters.find_first_of(s.substr(i, 1)) != string::npos) {
				if (c.size() > 0) r.push_back(c);
				c = "";
			}
			else c += s.substr(i, 1);
			
		}
		if (c.size() > 0) r.push_back(c);
		return r;
	}

	void consume_line(std::istream&is) {
		int c;
		do { c = is.get(); 
			//RDK_TRACE_PRINTF("Read %d=%c", c, c);
		} while( (c!='\n') && (c!=EOF));
	}
	
	void consumeWhitespace(std::istream&is) {
		while(1) {
			int next = is.peek();
			//RDK_TRACE_PRINTF("Next is %d=%c", next, next);
			
			if(next==EOF) 
				return;
			
			if(next=='#')
				consume_line(is);
			else
			if(!isspace(next))
				break;
			else
			is.get();		
		} 
	 }
	 
	std::string readCommentLines(std::istream&is, char commentChar) {
		std::string ws = "";
		while(1) {
			int next = is.peek();
			
			if(next==commentChar) {
				string line;
				std::getline(is,line);
				if(is.fail()) break;
				
				if(!line.empty()) 
					ws = ws + line.substr(1) + '\n';	
			} else if(isspace(next)) {
				is.get();	
			} else break;
		} 
		return ws;
	 }
	 
	 
	 
	bool startsWith(const std::string&s, const char*cs) {
		return !strncmp(s.c_str(), cs, strlen(cs));
	}
	
	
	bool endsWith(const std::string&s, const char*cs) {
		if(strlen(cs)>=s.length()) return false;
		return s.substr(s.length()-strlen(cs)) == cs;
	}

	std::string normalizeLibraryName(const std::string& l) {
		string libname = l;
		vector<string> vs = tokenize(libname, "/");
		libname = vs[vs.size()-1];
		// Strippo il "lib" iniziale
		if(startsWith(libname,"lib")) {
			libname = libname.substr(strlen("lib"));
		}
		
		// Strippo il ".so" finale
		if(endsWith(libname, ".so")) {
			libname = libname.substr(0,libname.length()-strlen(".so"));
		}
		// Strippo il ".dylib" finale
		if(endsWith(libname, ".dylib")) {
			libname = libname.substr(0,libname.length()-strlen(".dylib"));
		}
		
		// Rimetto il necessario
	#ifdef LINUX
		libname = string("lib") + libname + string(".so");
	#endif
	#ifdef MACOSX
		libname = string("lib") + libname + string(".dylib");
	#endif
	#ifdef CYGWIN
		libname = string("cyg") + libname + string(".dll");
	#endif
		return libname;
	}

	string trim(const string& s, const char* charsToTrim)
	{
		if (s == "") return "";
		size_t a = s.find_first_not_of(charsToTrim);
		size_t b = s.find_last_not_of(charsToTrim);
		if (a == string::npos || b == string::npos) return "";
		return s.substr(a, b-a+1);
	}

	string readEntireFile(cstr file) {
		ifstream ifs(file.c_str());
		ostringstream ss;
		ss << ifs.rdbuf();
		return ss.str();
	}

}} // ns

