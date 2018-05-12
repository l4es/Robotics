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

#ifndef RDK2_GEOMETRY_DSP
#define RDK2_GEOMETRY_DSP

#include <cmath>
#include <algorithm>
#include <sstream>
#include <assert.h>
#include <cmath>
#include <vector>
#include <rdkcore/geometry/dmatrix.h>

namespace RDK2 { namespace Geometry {
	
	/** Normalizes an array, values in @param x are copied in @param y
	   linearly mapped in nMin-nMax */
	template <typename X, typename Y>
	void normalize(const X * x, size_t length, Y * y, Y nMin, Y nMax) {
		if(!length) return;
		
		X cMin = x[0];
		X cMax = cMin;
		
		for(size_t i=0;i<length;i++) {
			cMin = std::min(cMin, x[i]);
			cMax = std::max(cMax, x[i]);
		}
	
		if(cMin==cMax) {
			for(size_t i=0;i<length;i++)
				y[i] = (Y) nMin;
		} else {
			for(size_t i=0;i<length;i++)
				y[i] = (Y) (nMin + (nMax-nMin)*(x[i]-cMin)/(cMax-cMin));
		}
	}

	/** Normalizes an array, values in @param x >0 are copied in @param y
	   scaled such that sum is  @param tot. It is safe to be x=y.*/
	template <typename X, typename Y>
	void normalize_sum(const X * x, size_t length, Y * y, Y tot) {
		Y sum = 0;
		for(size_t t=0;t<length;t++) 
			sum += x[t];
				
		//assert(sum!=0);
		
		double isum = sum!=0?tot/sum:1;
			
		for(size_t t=0;t<length;t++) 
			y[t] = x[t] * isum;
	}

	
	template<class Numeric>
	Numeric modulus(Numeric a, Numeric b) {
		return (a%b+b)%b;
	}
	
	/** Computes correlation between two arrays */
	template<class E, class F, class R>
	R mult(const E*r1, const F*r2, int size) {
		R r=0;
		for(int i=0;i< size ;i++) {
			E e = r1[i];
			F f = r2[i];
			r += (R) (e*f);
		}
		return r;
	}
	
	/** 
	Compute cross correlation between two arrays.
	This function was a pain to write.  
	result should have size deltaMax-deltaMin+1 
	*/
	template<class E, class F, class R>
	void cross_correlation(
		const E* r1,  int r1_size, 
		const F* r2,  int r2_size, 
		int deltaMin, int deltaMax, R* result) 
	{
		assert(deltaMax>deltaMin);
		int result_size = deltaMax - deltaMin + 1;
		for(int a=0;a<result_size;a++) {
			int diff = deltaMin + a;
			int r1_offset, r2_offset;
			
			if(diff>0) {
				r1_offset = diff;
				r2_offset = 0;
			} else {
				r1_offset = 0;
				r2_offset = -diff;
			}
			
			int len = std::min(r1_size-r1_offset, r2_size-r2_offset);
			
			result[a] = mult<E,F,R>( r1+r1_offset, r2+r2_offset, len);
		}
	}



	/** Normalizes the elements of a Matrix. See the other normalize(). */
	template <typename X, typename Y>
	DMatrix<Y> normalize(const DMatrix<X>& m, Y nMin, Y nMax) {
		DMatrix<Y> result(m.rows(), m.columns());
		normalize<X,Y>(m.getElems(), m.rows()*m.columns(), result.getElems(), nMin, nMax);
		return result;
	}

	/** Returns the elements of a dmatrix as a std::vector */
	template<class X>
	std::vector<X> getAsVector(DMatrix<X>& m) {
		std::vector<X> v;
		for(int i=0;i<m.rows();i++)
			for(int j=0;j<m.columns();j++)
				v.push_back(m[i][j]);
		return v;
	}
	
	/** Convolve matrix y (big) over matrix x (big). y should be squared with an odd number
	  of rows. Result  z  is as big as x.*/
	template<class X, class Y,class Z>
	void convolve(
		const DMatrix<X>& x, 
		const DMatrix<Y>& y,
		DMatrix<Z>& z) {
		// x and z of same size
		assert(z.columns() == x.columns());
		assert(z.rows() == x.rows());
		// y should be square and of odd size
		assert(y.rows() == y.columns());
		assert(y.rows()%2);
		
		for(int i=0; i<z.rows(); i++)
		for(int j=0; j<z.columns(); j++) {
			size_t m = y.rows() >> 1;
			DMatrix<X> around = x.around(i, j,  m);
			
			double value;
			y.product(around, value);
			z[i][j] = value;
		}
	}


	template<class X>
	X computeAverage(const std::vector<X>& values) {
		assert(values.size());
		X accumulator=0;
		for(size_t a=0;a<values.size();a++)
			accumulator+=values[a];
			
		return accumulator/values.size();
	}
	
	
	template<class X>
	X computeVariance(const std::vector<X>& values) {
		assert(values.size());
		X average = computeAverage(values);
		X accumulator=0;
		for(size_t a=0;a<values.size();a++)
			accumulator += square(values[a]-average);
		return accumulator/values.size();
	}
	
	template<class X>
	X computeSigma(const std::vector<X>& values) {
		assert(values.size());
		return sqrt(computeVariance(values));
	}
	

}} // ns

#endif
