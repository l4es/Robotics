
	template<class X>
	X& Viewport::w2b(DMatrix<X>&m, double x, double y) const 
	throw(std::invalid_argument) {
		if(!isInside(x,y)) {
			throw std::invalid_argument("");
		}
		int bx,by;
		basic_w2b(rmin, rmax, x,y, m.columns(), m.rows(), bx,by, false);
		return m[by][bx];
	}
	
	template<class X>
	const X& Viewport::w2b(const DMatrix<X>&m, double x, double y) const 
	throw(std::invalid_argument) {
		if(!isInside(x,y)) {
			throw std::invalid_argument("");
		}
		int bx,by;
		basic_w2b(rmin, rmax, x,y, m.columns(), m.rows(), bx,by, false);
		return m[by][bx];
	}
	
	template<class X>
	Point2d Viewport::b2w(const DMatrix<X>&m, int row, int column) const
	{
		double wx,wy;
		basic_b2w(rmin,rmax,column,row,m.columns(),m.rows(),wx,wy);
		return Point2d(wx,wy);
	}
	
	template<class X>
	Viewport Viewport::getCellBoundingBox(const DMatrix<X>&m, int row, int column) const
	{
		return basic_cellBB(rmin,rmax,column,row,m.columns(),m.rows());
	}


	template<class X, class PointType>
	X& Viewport::w2b(DMatrix<X>&m, const PointType& p) const 
	throw(std::invalid_argument) {
		return w2b(m,p.x,p.y);
	}
	
	template<class X, class PointType>
	const X& Viewport::w2b(const DMatrix<X>&m, const PointType& p) const 
	throw(std::invalid_argument) {
		return w2b(m,p.x,p.y);
	}
	
	
	template<class PointType>
	bool Viewport::isInside(const PointType& p) const {
		return  isInside((double)p.x, (double)p.y);
	}
	
	
	template<class X>
	Point2<double> Viewport::getCellSize(const DMatrix<X>&m) {
		return Point2<double>(
			width() / m.columns(),
			height() / m.rows()
		);
	}


