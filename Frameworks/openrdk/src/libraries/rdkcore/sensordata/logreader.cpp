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

#include "logreader.h"
#include "laserparser.h"

using namespace std;

namespace RDK2 { namespace SensorData {
	
	void Parser::setParam(const std::string&, const std::string&)
	{}
	
	LogReader::LogReader(istream*is, ostream*os): ls(is), os(os) {
		addParser(new LaserParser());
	}
	
	/** Parses a line of log. Returns 0 if ignored. */
	BaseSensorData* LogReader::parse(const std::string&s, std::string *error) {
		std::istringstream iss(s);
		std::string tag;
		if(!(iss>>tag)) {
			*error = s+": Could not read tag from string.";
			return 0;
		}
		Tag2Parser::iterator i = parsers.find(tag);
		if(i==parsers.end()) {
			Unknown * u = new Unknown();
			u->tag = tag;
			u->line = s;
			return u;
		}	
		return i->second->parse(s, error);
	}

	
	bool LogReader::write(const BaseSensorData*sd, std::string&line, std::string*error) {
		assert(sd);
		const Unknown * u = dynamic_cast<const Unknown*>(sd);
		if(u) {
			line = u->line;	
			return true;
		}
		Tag2Parser::iterator i = parsers.find(sd->tag);
		if(i==parsers.end()) {
			if(error) {
				*error = string("Could not find parser for tag ")+sd->tag;	
			}
			return false;
		}
		return i->second->write(sd, line, error);
	}
	



	
}} // end namespaces

