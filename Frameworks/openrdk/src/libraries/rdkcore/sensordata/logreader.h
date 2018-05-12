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

#ifndef H_SENSORDATA_LOGREADER
#define H_SENSORDATA_LOGREADER

#include <string>
#include <vector>
#include <iostream>
#include <map>
#include <rdkcore/geometry/point.h>
#include <rdkcore/geometry/dmatrix.h>
#include <rdkcore/textutils/linestream.h>

#include "sensordata.h"

namespace RDK2 { namespace SensorData {

struct Parser {
	virtual std::string getTag() = 0;
	virtual BaseSensorData * parse(
		const std::string& data,
		std::string * error=NULL) = 0; 

	virtual bool write(
		const BaseSensorData * input,
		std::string& data,
		std::string* error=NULL) = 0;
		
	virtual void setParam(const std::string& name, const std::string& line);  

	virtual ~Parser() {}	
};


class LogReader {
	public:
	
		LogReader(std::istream*is, std::ostream*os=NULL);
		
		void addParser(Parser*p) { parsers[p->getTag()] = p; }
		
		/** Parses a line of log. Returns 0 if ignored. */
		BaseSensorData* parse(const std::string&s, std::string *error=NULL);
		
		bool write(const BaseSensorData*sd, std::string&line, std::string*error=NULL);
		
		bool write(const BaseSensorData*sd) {
			if(!os) return false;
			std::string line, error;
			if(!write(sd, line, &error)) {
				std::cerr << "Warning: could not serialize "
					<< sd->tag << ": " << error << std::endl;
				return false;
			} else
			*os << line << std::endl;
			return true;
		}
	

	/* false if EOF or error */
	template<class X>
	bool getNext(X&c) {
		while(ls.readLine()) {
			std::string error;
			BaseSensorData * sd = parse(ls.getLine(), &error);
			if(!sd) {
				std::cerr << "Parsing error: " << error << std::endl;
				continue;
			}
			X * x = dynamic_cast<X*>(sd);
			if(x) {
				c = *x;
				delete sd;
				return true;
			} else {
				if(!os) { delete sd; continue; } 
				std::string line, error;
				if(!write(sd, line, &error)) {
					std::cerr << 
						"Warning: could not serialize " << sd->tag << ": " << error << std::endl;
				} else {
					*os << line << std::endl;
				}
				delete sd;
			}
		} 	
		return false;
	}

	template<class X>
	void getAll(std::vector<X>&v) {
		X x;
		while(getNext(x)) v.push_back(x);
	}
	

	private:
		RDK2::TextUtils::LineStream ls;
		std::ostream *os;
		
		typedef std::map<std::string, Parser*> Tag2Parser;
		Tag2Parser parsers;
		
	
};


}} // end namespaces

#endif

