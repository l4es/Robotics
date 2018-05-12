namespace RDK2 { namespace Geometry {

/// COSTRUCTORS amd reference counting

template <class X> DMatrix<X>::DMatrix(int n, int m) {
	assert(n>=1);
	assert(m>=1);
/*	if (n<1) n=1;
	if (m<1) m=1;*/
	nrows=n;
	ncols=m;
	this->elems=new X[nrows*ncols];
	this->mrows=new X* [nrows];
	for (int i=0;i<nrows;i++) mrows[i]=elems+ncols*i;
//	for (int i=0;i<nrows*ncols;i++) elems[i]=init;
	shares=new int;
	(*shares)=1;
}

template <class X> DMatrix<X>::DMatrix(const DMatrix& m) {
	shares=m.shares;
	elems=m.elems;
	nrows=m.nrows;
	ncols=m.ncols;
	mrows=m.mrows;
	(*shares)++;
}
template <class X> DMatrix<X>::~DMatrix() {
	if (--(*shares)) return;
	delete [] elems;
	delete [] mrows;
	delete shares;
}


template <class X> DMatrix<X>& DMatrix<X>::operator=(const DMatrix& m) {
	if (!--(*shares)) {
		delete [] elems;
		delete [] mrows;
		delete shares;
	}
	shares=m.shares;
	elems=m.elems;
	nrows=m.nrows;
	ncols=m.ncols;
	mrows=m.mrows;
	(*shares)++;
	return *this;
}


template <class X> void DMatrix<X>::detach() {
	DMatrix<X> aux(nrows,ncols);
	for (int i=0;i<nrows*ncols;i++) aux.elems[i]=elems[i];
	operator=(aux);
}


//////// ELEMENT ACCESS

template <class X> 
X DMatrix<X>::at(int i, int j, X zero) const {
	return isInside(i,j) ? mrows[i][j] : zero;
}

template <class X> 
X& DMatrix<X>::el(int i, int j) throw(std::invalid_argument) {
	if( isInside(i,j) )
		return mrows[i][j];
	throw std::invalid_argument("DMatrix::el(): out of bounds"); // XXX sputare stringa
}

template <class X> 
const X& DMatrix<X>::el(int i, int j) const throw(std::invalid_argument) {
	if( isInside(i,j) )
		return mrows[i][j];
	throw std::invalid_argument("DMatrix::el(): out of bounds"); // XXX sputare stringa
}
		 

template <class X> 
DMatrix<X> DMatrix<X>::aPieceOfYou(int i, int j, int m, int n, X zero) const {
	DMatrix<X> result(m,n);
	for(int u=0;u<m;u++)
		for(int v=0;v<n;v++)
			result[u][v] = at(i+u,j+v,zero);
	
	return result;	
}

template <class X> 
DMatrix<X> DMatrix<X>::around(int i, int j, int m, X zero) const {
	return aPieceOfYou(i-m,j-m,2*m+1,2*m+1,zero);
}


template <class X> 
bool DMatrix<X>::isInside(int i, int j) const {
	return i>=0 && j>=0 && i<nrows && j<ncols;
}

template <class X> 
void DMatrix<X>::setAll(const X& x) {
	willModify();
	for (int i=0;i<nrows;i++)
		for (int j=0;j<ncols;j++)
			(*this)[i][j]=x;
}

template<class X>
std::string DMatrix<X>::outputSpaceSeparated() const {
	std::ostringstream oss;
	oss << "   ";
	for(int i=0; i<nrows;i++)
		for(int j=0; j<ncols;j++)
			oss << at(i, j) << " ";
	oss << "   ";
	return oss.str();
}


template <class X> std::ostream& operator<<(std::ostream& os, const DMatrix<X> &m) {
	os << "{";
	for (int i=0;i<m.rows();i++) {
		if (i>0) os << ",";
		os << "{";
		for (int j=0;j<m.columns();j++) {
			if (j>0) os << ",";
			os << m[i][j];
		}
		os << "}";
	}
	return os << "}";
}

}} // end namespaces
