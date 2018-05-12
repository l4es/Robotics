namespace RDK2 { namespace Geometry {

// Numeric methods for dmatrix

template <class X> DMatrix<X> DMatrix<X>::inv() const {
	if (nrows!=ncols) throw DNotInvertibleMatrixException();
	DMatrix<X> aux1(*this),aux2(I(nrows));
	aux1.detach();
	for (int i=0;i<nrows;i++) {
		int k=i;
		for ( ;k<nrows&&aux1.mrows[k][i]==X(0);k++) ;
		if (k>=nrows) throw DNotInvertibleMatrixException();
		X val=aux1.mrows[k][i];
		for (int j=0;j<nrows;j++) {
			aux1.mrows[k][j]=aux1.mrows[k][j]/val;
			aux2.mrows[k][j]=aux2.mrows[k][j]/val;
		}
		if (k!=i) {
			for (int j=0;j<nrows;j++) {
				X tmp=aux1.mrows[k][j];
				aux1.mrows[k][j]=aux1.mrows[i][j];
				aux1.mrows[i][j]=tmp;
				tmp=aux2.mrows[k][j];
				aux2.mrows[k][j]=aux2.mrows[i][j];
				aux2.mrows[i][j]=tmp;
			}
		}
		for (int j=0;j<nrows;j++)
			if (j!=i) {
				X tmp=aux1.mrows[j][i];
				for (int l=0;l<nrows;l++) {
					aux1.mrows[j][l]=aux1.mrows[j][l]-tmp*aux1.mrows[i][l];
					aux2.mrows[j][l]=aux2.mrows[j][l]-tmp*aux2.mrows[i][l];
				}
			}
	}
	return aux2;
}

template <class X> const X DMatrix<X>::det() const {
	if (nrows!=ncols) throw DNotSquareMatrixException();
	DMatrix<X> aux(*this);
	X d=X(1);
	aux.detach();
	for (int i=0;i<nrows;i++) {
		int k=i;
		for ( ;k<nrows&&aux.mrows[k][i]==X(0);k++) ;
		if (k>=nrows) return X(0);
		X val=aux.mrows[k][i];
		for (int j=0;j<nrows;j++) {
			aux.mrows[k][j]/=val;
		}
		d=d*val;
		if (k!=i) {
			for (int j=0;j<nrows;j++) {
				X tmp=aux.mrows[k][j];
				aux.mrows[k][j]=aux.mrows[i][j];
				aux.mrows[i][j]=tmp;
			}
			d=-d;
		}
		for (int j=i+1;j<nrows;j++){
			X tmp=aux.mrows[j][i];
			if (!(tmp==X(0)) ){
				for (int l=0;l<nrows;l++) {
					aux.mrows[j][l]=aux.mrows[j][l]-tmp*aux.mrows[i][l];
				}
				//d=d*tmp;
			}
		}
	}
	return d;
}

template <class X> DMatrix<X> DMatrix<X>::transpose() const {
	DMatrix<X> aux(ncols, nrows);
	for (int i=0; i<nrows; i++)
		for (int j=0; j<ncols; j++)
			aux[j][i]=mrows[i][j];
	return aux;
}

template <class X> DMatrix<X> DMatrix<X>::operator*(const DMatrix<X>& m) const {
	if (ncols!=m.nrows) throw DIncompatibleMatrixException();
	DMatrix<X> aux(nrows,m.ncols);
	for (int i=0;i<nrows;i++)
		for (int j=0;j<m.ncols;j++){
			X a=0;
			for (int k=0;k<ncols;k++)
				a+=mrows[i][k]*m.mrows[k][j];
			aux.mrows[i][j]=a;
		}
	return aux;
}

template <class X> DMatrix<X> DMatrix<X>::operator+(const DMatrix<X>& m) const {
	if (ncols!=m.ncols||nrows!=m.nrows) throw DIncompatibleMatrixException();
	DMatrix<X> aux(nrows,ncols);
	for (int i=0;i<nrows*ncols;i++) aux.elems[i]=elems[i]+m.elems[i];
	return aux;
}

template <class X> DMatrix<X> DMatrix<X>::operator-(const DMatrix<X>& m) const {
	if (ncols!=m.ncols||nrows!=m.nrows) throw DIncompatibleMatrixException();
	DMatrix<X> aux(nrows,ncols);
	for (int i=0;i<nrows*ncols;i++) aux.elems[i]=elems[i]-m.elems[i];
	return aux;
}

template <class X> DMatrix<X> DMatrix<X>::operator*(const X& e) const {
	DMatrix<X> aux(nrows,ncols);
	for (int i=0;i<nrows*ncols;i++) aux.elems[i]=elems[i]*e;
	return aux;
}

template <class X> bool DMatrix<X>::operator==(const DMatrix& m) const {
	if (ncols!=m.ncols||nrows!=m.nrows) throw DIncompatibleMatrixException();
	int i=0;
	while (i<nrows*ncols && m.elems[i] == elems[i])
	{
		++i;
	}
	return i == nrows*ncols;
}


template <class X>
void DMatrix<X>::multAll(const X& x) {
	willModify();
	for (int i=0;i<nrows;i++)
		for (int j=0;j<ncols;j++)
			(*this)[i][j] *=x;
}


template <class X>
template <class Y, class Z>
void DMatrix<X>::product(const DMatrix<Y>& m, Z&result) const {
	if (ncols!=m.columns()||nrows!=m.rows()) throw DIncompatibleMatrixException();
	result = 0;
	for (int i=0;i<nrows;i++)
		for (int j=0;j<ncols;j++)
		result += (Z) (mrows[i][j] * m[i][j]);
}


template <class X> DMatrix<X> DMatrix<X>::I(int n) {
	assert(n);
	DMatrix<X> aux(n,n);
	aux.setAll(0);
	for (int i=0;i<n;i++) aux[i][i]=X(1);
	return aux;
}



template <class X>
DMatrix<X> MRot(X theta) {
	DMatrix<X> matrix(2,2);
	matrix[0][0] =  cos(theta);
	matrix[0][1] = -sin(theta);
	matrix[1][0] =  sin(theta);
	matrix[1][1] =  cos(theta);
	return matrix;
}

}} // end namespaces
