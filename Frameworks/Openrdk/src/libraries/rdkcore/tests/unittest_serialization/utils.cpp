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

#include <fstream>
#include <iostream>
#include "utils.h"

using namespace std;

bool compareToFile(const std::string& buffer,const std::string& file) {
	ifstream ifs(file.c_str());
	
	if(!ifs) {
		// File does not exists
		cerr << "test1: Creating file " << file << endl;
		ofstream ofs(file.c_str());
		ofs.write(buffer.data(), buffer.length());
		ofs.close();
	} else {
		// Read from file
		string f; int c; while(c=ifs.get(), ifs) f.push_back(c);
		// and compare
		if(f!=buffer) {
			cerr << "test1: file " << file << " length is " << f.length() 
				  << ", buffer length = " << buffer.length() << endl;
			return false;	
		}
	}
	return true;
}
