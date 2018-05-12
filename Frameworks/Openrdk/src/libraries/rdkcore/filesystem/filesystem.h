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

#ifndef H_FILESYSTEM
#define H_FILESYSTEM

#include <string>
#include <stdexcept>
#include <fstream>

#include <rdkcore/textutils/textutils.h>

/** 
 * @file
 *
 * @brief Some common functions for manipulating files and filenames. 
 */
namespace RDK2 { namespace Filesystem {

	using namespace std;
	using namespace RDK2::TextUtils;

/// Opening
//@{
	/** Opens file for reading, or throws an exception. */
	void fsOpenOrException(ifstream&ifs, cstr filename) throw(runtime_error);
	
	/** Opens file for writing, or throws an exception. */
	void fsOpenOrException(ofstream&ofs, cstr filename) throw(runtime_error);
//@}

/// Filenames 
//@{
	/** Concat. two paths. */
	string fsCat(cstr path1, cstr path2);
	
	/** Normalizes a filename, runtime_error on problems. */
	string fsRealpath(cstr s) throw (runtime_error);

	/** Last part of path */
	string fsOnlyFilename(cstr path) throw(runtime_error);
	
	/** Returns name of the directory */
	string fsDirname(cstr file) throw(runtime_error);
	
//@}

/// Misc
//@{
	/** Create the parents of path (a la makeparents)*/
	void fsMakeParents(cstr path) throw(runtime_error);
	
	/// Returns true if file exists
	bool fsExists(cstr file) throw();
	
	/// Returns true if file1 is newer than file2
	bool fsNewerThan(cstr file1, cstr file2) throw(runtime_error);
	
	/// reads a file into a string
	string fsReadFile(cstr file) throw(runtime_error); 
	
//@}

	bool fsDirContent(const std::string& directory, std::vector<std::string>& dirContent);

}} // namespace RDK2::Filesystem

#endif
