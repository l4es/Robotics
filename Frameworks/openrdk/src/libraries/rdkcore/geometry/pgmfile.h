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

#ifndef RDK2_PGMFILE
#define RDK2_PGMFILE

#include <stdexcept>
#include <rdkcore/geometry/dmatrix.h>
#include <rdkcore/geometry/viewport.h>

namespace RDK2 { namespace Geometry {

	using std::runtime_error;
	using std::invalid_argument;
	
	/** Alcune funzioni per caricare e salvare i pgm (sia P2 che P5), 
	cercando di unire il codice ripetuto fra DMatrix e RMapImage.
	Puo' darsi che questa roba non vada in Geometry (che bello subversion
	che si possono spostare i file. */
	
	struct PGMFile {
		enum Type { P2, P5 } type;
		
		/** The comment section in the file. */
		std::string comment;
		
		int maxVal,width,height;
		DMatrix<int> buffer;
		
		/** Opens a PGM file, throws an exceptions if there are problems. */
		PGMFile(const std::string&file) throw(runtime_error) {
			load(file);
		}
		
		PGMFile();
		
		void load(std::istream&is) throw(runtime_error);
		void load(const std::string&file) throw(runtime_error);
		bool load_(const std::string&file);

		/* Returns a viewport for the image. If meters_per_pixel==0 , it tries
		to parse "mapResolution XXX" from the comment. */
		RDK2::Geometry::Viewport getViewport(double meters_per_pixel=0) 
			throw (runtime_error, invalid_argument);
		
		bool readResolution(double&);
		
	};
	
}} // ns

#endif
