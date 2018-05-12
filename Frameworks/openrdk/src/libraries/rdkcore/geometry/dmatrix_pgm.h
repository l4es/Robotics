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

#ifndef RDK2_DMATRIX_PGM
#define RDK2_DMATRIX_PGM

#include <iostream>
#include <fstream>
#include <rdkcore/geometry/utils.h>

#include <rdkcore/textutils/textutils.h>

#include "dmatrix.h"
#include "dsp.h"

namespace RDK2 { namespace Geometry {
	
template<typename Numeric>
bool loadPGM(std::istream&is, DMatrix<Numeric>& matrix);

template<typename Numeric>
bool loadPGM(const char*file, DMatrix<Numeric>& matrix);

/** Your matrix will *not* be normalized. */
template <typename Numeric>
bool savePGM(const DMatrix<Numeric>&matrix, std::ostream&os);

/** Your matrix will be normalized on black-white. */
template <class Numeric>
bool savePGM(std::string file, const DMatrix<Numeric>&matrix);

// Implementation

template<typename Numeric>
bool loadPGM(std::istream&is, DMatrix<Numeric>& matrix) {
	std::string magicNumber;
	is >> magicNumber;
	
	if(magicNumber!= "P5") {
		// RDK_ERROR_STREAM("I expected a magic of P5, instead of '"<<magicNumber<<"'");
		return false;
	}
	
	RDK2::TextUtils::consumeWhitespace(is);
	
	int width = -1, height = -1, maxVal = -1;
	
	if(!(is>>width)) {
		// RDK_ERROR_STREAM("Could not read width");
		return false;
	}	
	
	//RDK_TRACE_PRINTF("Read width=%d", width);
	RDK2::TextUtils::consumeWhitespace(is);
	
	if(!(is>>height)) {
		// RDK_ERROR_STREAM("Could not read height");
		return false;
	}	
	
	//RDK_TRACE_PRINTF("Read height=%d", height);
	RDK2::TextUtils::consumeWhitespace(is);
	
	if(!(is>>maxVal)) {
		// RDK_ERROR_STREAM("Could not read maxVal");
		return false;
	}	
	
	// RDK_TRACE_PRINTF("Read maxVal=%d", maxVal);
	
	if(width<=0 || height <=0) {
		// RDK_ERROR_PRINTF("Bad width, height = %d, %d", width, height);
		return 0;
	}
		
	#define EXPECT_MAXVAL 255
	if(maxVal!=EXPECT_MAXVAL) {
		// RDK_ERROR_PRINTF("I expected a maxVal of %d instead of %d.", EXPECT_MAXVAL, maxVal);
		return 0;
	}
	
	is.get();
	
	matrix = DMatrix<Numeric>(height, width);
	
	for(int y=0;y<height;y++)
		for(int x=0;x<width;x++)  {
			int c = is.get();
			matrix[y][x] = c;
		}
		
	return true;
}

template<typename Numeric>
bool loadPGM(const char*file, DMatrix<Numeric>& matrix) {
	std::ifstream ifs(file);
	if(!ifs) {
		//RDK_ERROR_STREAM("Could not open file  '"<<file<<"'.");
		return false;
	}
	
	return loadPGM(ifs, matrix);
}


template <typename Numeric>
bool savePGM(const DMatrix<Numeric>&matrix, std::ostream&os) {
	int width  = matrix.columns();
	int height = matrix.rows();
	
		os << "P5"             << std::endl
		<< "# DMatrix "        << std::endl
		<< width <<" "<<height << std::endl
		<< "255"               << std::endl;
	
	for(int y=0;y<height;y++)
	for(int x=0;x<width;x++)
		os.put(matrix[y][x]);
		
	return !os.fail();
}

template <class Numeric>
bool savePGM(std::string file, const DMatrix<Numeric>&matrix) {
	std::ofstream ofs(file.c_str());
	DMatrix<unsigned char> m = 
		RDK2::Geometry::normalize(matrix, (unsigned char)0,(unsigned char)255);
	return savePGM(m, ofs);
}

}} // end namespaces

#endif
