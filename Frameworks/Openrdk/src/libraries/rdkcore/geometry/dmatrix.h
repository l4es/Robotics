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

#ifndef RDK2_GEOMETRY_DMATRIX
#define RDK2_GEOMETRY_DMATRIX

#include <assert.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <rdkcore/textutils/textutils.h>

#include "utils.h"
#include "point2.h"

namespace RDK2 { namespace Geometry {

class DNotInvertibleMatrixException: public std::exception {};
class DIncompatibleMatrixException: public std::exception {};
class DNotSquareMatrixException: public std::exception {};

/**
Attenzione al costruttore: rows,columns == height,width

Attenzione ai metodi di accesso:
	at(i,j) � l'elemento alla riga i e colonna j

Se state usando DMatrix per rappresentare mappe o altre cose
spaziali, vi conviene usare la classe Viewport che ha implementato
per bene i metodi per passare dalle coordinate mondo alla cella
delle griglia e viceversa. (non ve ne pentirete)

Se volete usare x,y vi conviene accedere in questo modo:
	at(y, x)
ovvero y->righe, x->colonne.

Questa classe implementa il copy-on-write. Da utente, � tutto nascosto
ai vostri occhi. Se aggiungete metodi che cambiano i dati,
ricordate di chiamare willModify() prima di modificare effettivamente.
*/

template <class X> class DMatrix {
	public:
		/** Default constructor X() is called for every cell. */
		DMatrix(int rows=1, int columns=1);
		~DMatrix();

		DMatrix(const DMatrix&);
		DMatrix& operator=(const DMatrix&);

		X * operator[](int i) {
			willModify();
			return mrows[i];
		}

		const X * operator[](int row) const { return mrows[row]; }

		int rows() const { return nrows; }
		int columns() const { return ncols; }


		/** Returns a copy of the sub-matrix i,j-i+m,j+n.
		   Can go over the borders, will be padded with zero. */
		DMatrix<X> aPieceOfYou(int i, int j, int m, int n, X zero=0) const;

		/** Returns the 2m+1 square sub-matrix around point i,j */
		DMatrix<X> around(int i, int j, int m, X zero=0) const;

		/** Returns value at row=i,column=j of zero if out of borders */
		X at(int i, int j, X zero=0) const;

		/** Access element at row=i=y,column=j=x; throws exception if out of borders */
		// AC: il nome el (per "elemento") � brutto ma non ho
		// voluto toccare il metodo sopra
		X& el(int i, int j) throw(std::invalid_argument);
		const X& el(int i, int j) const throw(std::invalid_argument);

		/** Equivalent to el(p.y, p.x) */
		X& el(const Point2<int>&p) throw(std::invalid_argument){
			return el(p.y, p.x);
		}

		const X& el(const Point2<int>&p) const throw(std::invalid_argument){
			return el(p.y, p.x);
		}

		/** Checks whether point is inside the matrix. */
		bool isInside(int i, int j) const;

		/** Equivalent to isInside(p.y, p.x) */
		bool isInside(const Point2<int>&p) const {
			return isInside(p.y, p.x);
		}

		/** Set all elements to value */
		void setAll(const X& x);

		/** Applies an operation to all elements */
		template<class Op> void applyAll(Op&op) {
			willModify();
			for(int a=0;a<nrows*ncols;a++)
				op(elems[a]);
		}

		// for(Point2i c = grid.begin(); c != grid.end(); c = grid.next(c) )  {
		Point2<int> begin() { return Point2<int>(0,0); }
		Point2<int> end() { return Point2<int>(-1,-1); }
		Point2<int> next(const Point2<int>&prev) {
			if(prev.x<columns()-1)
				return Point2<int>(prev.x+1,prev.y);
			else {
				if(prev.y<rows()-1)
					return Point2<int>(0,prev.y+1);
				else
					return end();
			}
		}

	/// These methods do the copy-on-write
	protected:
		/// Call this method before modifying the data.
		void willModify() {
			if ((*shares)>1)
				detach();
		}

		void detach();

	public:
///
/// For matrixes holding numbers
///
		/** Correlation (sum of product of corresponding elements).
		   Note that return value is passed as a reference beacause of compiler's
		   problems with templates. */
		template <class Y, class Z>
		void product(const DMatrix<Y>&, Z&z) const;

		/** Multiplies all elements by a value. */
		void multAll(const X& x);

		DMatrix transpose() const;
		const X det() const;
		DMatrix inv() const;
		//bool isPositiveDefinite() const;

		/// Returns the identity matrix.
		static DMatrix I(int n);

		DMatrix operator*(const DMatrix&) const;
		DMatrix operator+(const DMatrix&) const;
		DMatrix operator-(const DMatrix&) const;
		DMatrix operator*(const X&) const;
		bool operator==(const DMatrix&) const;

	protected:
		int nrows,ncols;
		X * elems;
		X ** mrows;

		int * shares;

	public:
		// Workaround for friend templates classes (DMatrix<X>!=DMatrix<Y>)
		X* getElems() { willModify(); return elems; }
		const X* getElems() const { return elems; }

	public:
		/** Output as space separated row1 .. rowN **/
		std::string outputSpaceSeparated() const;

};

typedef DMatrix<double> DMatrixD;

}} // end namespaces (we need to #include!)

#include "dmatrix_imp.hpp"
#include "dmatrix_numeric.hpp"

namespace RDK2 { namespace Geometry {

template <class X> std::ostream& operator<<(std::ostream& os, const DMatrix<X> &m);

/** Rotation matrix 2x2*/
template <class X> DMatrix<X> MRot(X theta);


bool parseMatrix(
	const std::vector<std::string>& sv, int start_index, int rows, int columns,
	DMatrix<double> &d, std::string*error=NULL);



}} // end namespaces

#endif
