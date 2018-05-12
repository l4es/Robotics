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

#include "streamsink.h"
#include <sstream>
#include <vector>
#include <cstdlib>
using std::ostringstream;


namespace Logging {

StreamSink::StreamSink(std::ostream*os_, bool usecolor_, bool manage_)
	: os(os_), manage(manage_) , usecolor(usecolor_) {

	char * env = getenv("COLUMNS");
	columns = env ? atoi(env) : DEFAULT_COLUMNS;

	if(columns<COLUMNS_LIMIT) {
		*os << "Found columns = " << columns << ", using " << DEFAULT_COLUMNS << " instead." <<std::endl;
		columns = DEFAULT_COLUMNS;
	}

	// *os << "Console started ("<<columns<<" columns) ." << std::endl;
}

string pad(const string&r, unsigned int l, const string&t=" ") {
	string s(r);
	while(s.length()<l) s=t+s;
	return s;
}

string chop(const string&r, unsigned int len) {
	return r.length() <= len ? r : r.substr(0, len);
}

string align(const string& r, unsigned int len) {
	return chop(pad(r,len), len);
}

template<class T> string toString(const T&t) {
	std::ostringstream oss;
		oss << t;
	return oss.str();
}

using namespace std;

int splitString(const string& input, 
       const string& delimiter, vector<string>& results, 
       bool includeEmpties);

void splitStringToSize(string s, size_t size, vector<string>& results) {
	while(s.length()>size) {
		string start = s.substr(0,size-3) + " ->";
		results.push_back(start);
		s = s.substr(size-3);
	}
	while(s.length()<size) s.append(" ");
	results.push_back(s);
}

void StreamSink::handle(const LogMessage&l) {
	string TERM_RESET = "\e[0m";
	const int LEN_FILE   = 30;
	const int LEN_STAT   =  4;
	const int LEN_LINE   =  3;

	ostringstream oss; // XXX
	
	ostringstream left;
	/*
	left
		<< align(         l.module, LEN_MODULE ) << "|"
		<< align(           l.file, LEN_FILE   ) << ":"
		<< align( toString(l.line), LEN_LINE   ) << "|"; */


	const char * file = l.file.c_str();
	const char * filename = file + l.file.length();
	for(;filename>file && (*(filename-1) != '/');filename--) ;
	
	left
		<< align(l.module+"|"+filename, LEN_FILE ) << ":"
		<< align( toString(l.line), LEN_LINE   ) << "|"; 
		
	if(l.level==Error)
		*os << "\a";
		
	string symbol, color;
	switch(l.level) {
		case(Trace): symbol = " ~  "; color = "1;37;47"; break;
		case(Debug): symbol = " db "; color = "1;37;45"; break;
		case(Info ): symbol = "    "; color = "1;37;42"; break;
		case(Error): symbol = " :( "; color = "1;37;41"; break;
		case(Warning): symbol = " !! "; color = "1;37;43"; break;
		case(Fatal): symbol = " @@ "; break;
	}
	
	color = string("\e[") + color + string("m");
	
	if(!usecolor) {
		left << align(symbol, LEN_STAT);
	}	
	
	string leftString = left.str();
	string::size_type header = leftString.length();
	string::size_type rest = columns-header;

	vector<string> lines1;
	splitString(l.message,"\n", lines1, true);
	
	vector<string> lines2;

	for(size_t i=0;i<lines1.size();i++)
		splitStringToSize(lines1[i], rest, lines2);

	for(size_t i=0;i<lines2.size();i++) {
		string line = lines2[i];

		if(i!=0) {
			for(size_t a=0;a<header;a++) oss << " "; 
		} else oss << leftString;
	
		if(usecolor) oss << color;
 		oss << line;
		if(usecolor) oss << TERM_RESET;
		oss << std::endl;
	}
	
	*(this->os) << oss.str() << std::flush;
}

StreamSink::~StreamSink() {
	if(manage) delete os;
}

int splitString(const string& input, 
       const string& delimiter, vector<string>& results, 
       bool includeEmpties)
{
	int iPos = 0;
	int newPos = -1;
	int sizeS2 = (int)delimiter.size();
	int isize = (int)input.size();

	if( 
	    ( isize == 0 )
	    ||
	    ( sizeS2 == 0 )
	)
	{
	    return 0;
	}

	vector<int> positions;

	newPos = input.find (delimiter, 0);

	if( newPos < 0 ) { 
	results.push_back(input);
	    return 1; 
	}

	int numFound = 0;

	while( newPos >= iPos )
	{
	    numFound++;
	    positions.push_back(newPos);
	    iPos = newPos;
	    newPos = input.find (delimiter, iPos+sizeS2);
	}

	if( numFound == 0 )
	{
	    return 0;
	}

	for( int i=0; i <= (int)positions.size(); ++i )
	{
	    string s("");
	    if( i == 0 ) 
	    { 
	        s = input.substr( i, positions[i] ); 
	    }
	    int offset = positions[i-1] + sizeS2;
	    if( offset < isize )
	    {
	        if( i == (int) positions.size() )
	        {
	            s = input.substr(offset);
	        }
	        else if( i > 0 )
	        {
	            s = input.substr( positions[i-1] + sizeS2, 
	                  positions[i] - positions[i-1] - sizeS2 );
	        }
	    }
	    if( includeEmpties || ( s.size() > 0 ) )
	    {
	        results.push_back(s);
	    }
	}
	return numFound;
	}

}
