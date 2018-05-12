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

#ifndef RDK2_GEOMETRY_OUTPUT
#define RDK2_GEOMETRY_OUTPUT

#include <string>
#include <iostream>

#include "dmatrix.h"
#include "point.h"
#include "otherpoints.h"

namespace RDK2 { namespace Geometry {
	
	/** Metodi di output per scrivere su file vettori di punti (leggibili da gnuplot) */	
	void output(const std::string& file, const AVV& v, bool normalize=false, double scale=1);
	void output(const std::string& file, const AppliedVector*v, int size, bool normaliza=false, double scale=1);

	void output(const std::string& file, const OPVec& v, double scale=1);
	void output(const PPVec&vec, const char*filename);

	void outputNormal(const std::string& file, const AVV& v,
		bool normalize=false, double scale=1);
	void outputNormal(std::ostream&os, const AppliedVector*v, int size,
		bool normalize=false, double scale=1);
	void outputNormal(std::ostream&os, const AVV& v, 
		bool normalize=false, double scale=1);
	void outputNormal(const std::string& file, const AppliedVector*v, int size,
		bool normalize=false, double scale=1);
	void outputNormal(const std::string& file, const OPVec& v);
			
	/** Draws gaussian ellipse (maybe broken) */
	void outputGaussian(std::ostream& ofs, Point2d estimate, DMatrix<double> variance);
	
	
}} // ns	

#endif
