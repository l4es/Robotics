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

#ifndef RDK2_DMATRIX_ALGEBRA
#define RDK2_DMATRIX_ALGEBRA

#include "dmatrix.h"
#include "point.h"

namespace RDK2 { namespace Geometry {

	/** Returns eigenvectors and eigenvalues for a 2x2 matrix */
	bool eig22_gsl(
		const DMatrix<double>&m, 
		double& lambda1, Point2d& v1, 
		double& lambda2, Point2d& v2);
	
	DMatrix<double> reassembleFromEig(
		double lambda1, Point2d v1, 
		double lambda2, Point2d v2);
	
	bool isDefinitePositive(const DMatrixD& m);
	bool isSemiDefinitePositive(const DMatrixD& m);
	
	/** Shortcut to build a 2x2 matrix */	
	template<class X>
	DMatrix<X> matrix22(const X& a,const X& b, const X& c, const X& d) {
		DMatrix<X> t(2,2);
		t[0][0] = a;
		t[1][0] = c;
		t[0][1] = b;
		t[1][1] = d;
		return t;
	}

	/** Shortcut to build a 3x3 matrix from a scalar and 2x2 */	
	template<class X>
	DMatrix<X> matrix33(X&topleft, const DMatrix<X>&d22, X pad=0) {
		DMatrix<X> t(3,3);
		t[0][0] = topleft;
		t[0][1] = t[0][2] = pad;
		t[1][0] = t[2][0] = pad;
		t[1][1] = d22[0][0];
		t[2][1] = d22[1][0];
		t[1][2] = d22[0][1];
		t[2][2] = d22[1][1];
		return t;
	}

	template<class X>
	DMatrix<X> matrix33( const DMatrix<X>&topleft22,X&bottomright, X pad=0) {
		DMatrix<X> t(3,3);
		
		t[0][0] = topleft22[0][0];
		t[1][0] = topleft22[1][0];
		t[0][1] = topleft22[0][1];
		t[1][1] = topleft22[1][1];

		t[2][1] = pad;
		t[2][0] = pad;
		t[1][2] = pad;
		t[0][2] = pad;
		
		t[2][2] = bottomright;
		
		return t;
	}

	bool getWhiteningMatrix22(const DMatrix<double>&m, DMatrix<double>&w);

	/** 3x3 rotation matrix around Z */
	DMatrix<double> rot3z(double theta);

	/// DEPRECATED
	/** Measure of uncertainty for covariance matrix */
	double uncertainty(const DMatrix<double>&m);

}} // namespaces

#endif
