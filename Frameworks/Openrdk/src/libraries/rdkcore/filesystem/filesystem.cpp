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

#include <sstream>
#include <cstdio>
#include <errno.h>
#include <cstdlib>

#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <dirent.h>

#include <libgen.h>

#include "filesystem.h"

namespace RDK2 { namespace Filesystem {

	void fsMakeParents(cstr /*path*/) throw(runtime_error) {
		#warning Implement fsMakeParents
	}

	string fsOnlyFilename(cstr path) throw(runtime_error) {
		string s((const char*)basename((char*)path.c_str()));
		return s;
	}

	void fsOpenOrException(ifstream&ifs, cstr filename) throw(runtime_error) {
		ifs.open(filename.c_str());
		if(!ifs)
			throw runtime_error(string("Could not open file ")+quote(filename)+" for reading.");
	}

	void fsOpenOrException(ofstream&ofs, cstr filename) throw(runtime_error) {
		ofs.open(filename.c_str());
		if(!ofs)
			throw runtime_error(string("Could not open file ")+quote(filename)+" for writing.");
	}

	string fsCat(cstr path1, cstr path2) {
		return fsRealpath(path1+"/"+path2);
	}

	string fsRealpath(cstr s) throw (runtime_error) {
		// the use of PATH_MAX is deprecated
		// please fix this
		// FIXME-LM
		char resolved_path[4096];

		if(NULL==realpath(s.c_str(), resolved_path)) {
			throw runtime_error(string("Could not get realpath for file ")+quote(s)
			+", reason = " + strerror(errno));
		} else return string(resolved_path);
	}

	double modificationTime(cstr file) {
		struct stat s;
		if(stat(file.c_str(),&s)) {
			throw runtime_error(string("Could not get modification time ")
			+"for file " + quote(file)+"; error: " + strerror(errno));
		}
#ifdef LINUX
		return s.st_mtime;
#endif

#ifdef MACOSX
		return s.st_mtimespec.tv_sec;
#endif
		
#ifdef CYGWIN
		return s.st_mtime;
#endif
	}

	bool fsNewerThan(cstr file1, cstr file2) throw(runtime_error) {
		return modificationTime(file1) > modificationTime(file2);
	}

	bool fsExists(cstr file) throw() {
		struct stat s;
		return 0 == stat(file.c_str(),&s);
	}

	string fsReadFile(cstr file) throw(runtime_error) {
		ifstream ifs; fsOpenOrException(ifs, file);
		ostringstream oss; // XXX non c'� bisogno
		char c;
		 while(ifs.get(c)) oss.put(c);
		return oss.str();
	}

	/** Returns name of the directory */
	string fsDirname(cstr file) throw(runtime_error) {
		char * dir = dirname((char*) file.c_str());
		if(!dir)
			throw runtime_error(string("Could not get dirname for ")+file);
		return string(dir);
	}

bool fsDirContent(const string& directory, vector<string>& dirContent)
{
	DIR* dip = opendir(directory.c_str());
	if (!dip) return false;

	dirContent.clear();
	struct dirent *dit;
	while ((dit = readdir(dip))) {
		dirContent.push_back(string(dit->d_name));
	}
	
	if (closedir(dip) == -1) return false;
	return true;
}

}} // namespace RDK2::Filesystem;
