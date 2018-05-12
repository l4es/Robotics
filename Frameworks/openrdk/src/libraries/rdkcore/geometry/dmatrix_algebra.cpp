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

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_eigen.h>
#include "dmatrix_algebra.h"

namespace RDK2 { namespace Geometry {

DMatrix<double> rot3z(double theta) {
	DMatrix<double> m(3,3);
	m[0][0] = cos(theta);
	m[0][1] = -sin(theta);
	m[1][0] = sin(theta);
	m[1][1] = cos(theta);
	
	m[0][2] = 0;
	m[1][2] = 0;
	m[2][0] = 0;
	m[2][1] = 0;
	m[2][2] = 1;
	
	return m;
}


	bool isDefinitePositive(const DMatrixD& m) {
		double l[2]; Point2d v[2];
		RDK2::Geometry::eig22_gsl(m, l[0], v[0], l[1],v[1]);
		return l[0] > 0 && l[1] > 0;
	}
	
	bool isSemiDefinitePositive(const DMatrixD& m) {
		double l[2]; Point2d v[2];
		RDK2::Geometry::eig22_gsl(m, l[0], v[0], l[1],v[1]);
		return l[0] >= 0 && l[1] >= 0;
	}
	
	DMatrix<double> reassembleFromEig(
		double lambda1, Point2d v1, 
		double lambda2, Point2d v2) {
			
		DMatrixD P(2,2);
		P.el(0,0) = v1.x; P.el(0,1) = v2.x;
		P.el(1,0) = v1.y; P.el(1,1) = v2.y;
		
		DMatrixD D(2,2);
		D.el(0,0) = lambda1; D.el(0,1) = 0;
		D.el(1,0) = 0; D.el(1,1) = lambda2;
		
		return P * D * P.inv();
	}

	
void copy(const DMatrix<double>&m, gsl_matrix * gslm) {
	for(int i=0;i<m.rows();i++)
	for(int j=0;j<m.columns();j++)
		gsl_matrix_set (gslm, i,j, m[i][j]);
}

double uncertainty(const DMatrixD&m) {
	double l[2] = {0,0};
	Point2d v[2];
	if(eig22_gsl(m, l[0],v[0],l[1],v[1])) {
		return sqrt(std::max(l[0],l[1]));
	} 
	return 0;
}
bool eig22_gsl(
	const DMatrix<double>&m, 
	double& lambda1, Point2d& v1, 
	double& lambda2, Point2d& v2) {
	
	assert(m.rows()==2);
	assert(m.columns()==2);
	
	gsl_matrix            *matrix = gsl_matrix_alloc(2,2);
	gsl_vector              *eval = gsl_vector_alloc(2);
	gsl_matrix              *evec = gsl_matrix_alloc(2,2);
	gsl_eigen_symmv_workspace *ws = gsl_eigen_symmv_alloc(2);
	
	copy(m, matrix);
	// XXX errors?
	gsl_eigen_symmv(matrix, eval, evec, ws);
	
	lambda1 = gsl_vector_get(eval, 0);
	lambda2 = gsl_vector_get(eval, 1);
	v1 = Point2d(gsl_matrix_get(evec, 0, 0),gsl_matrix_get(evec, 1, 0));
	v2 = Point2d(gsl_matrix_get(evec, 0, 1),gsl_matrix_get(evec, 1, 1));
	
	gsl_vector_free(eval);
	gsl_matrix_free(evec);
	gsl_matrix_free(matrix);
	gsl_eigen_symmv_free(ws);
	return true;
}

bool getWhiteningMatrix22(const DMatrix<double>&m, DMatrix<double>&w) {
	Point2d v[2];
	double l[2];

	if(m.rows()!=2 || m.columns() != 2) {
		return false;
	}
	
	if(!eig22_gsl(m, l[0], v[0], l[1], v[1])) {
		return false;
	}
	
	DMatrix<double> u(2,2);
	u[0][0] = v[0].x; u[0][1] = v[1].x;
	u[1][0] = v[0].y; u[1][1] = v[1].y;
	
	DMatrix<double> d(2,2);
	d[0][0] = sqrt(1/l[0]); d[0][1] = 0;
	d[1][0] = 0;            d[1][1] = sqrt(1/l[1]); 
	
	w = d * u.inv();
	
	return true;
}

}}
