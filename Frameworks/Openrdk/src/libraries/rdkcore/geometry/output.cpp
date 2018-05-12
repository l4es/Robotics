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

#include <fstream>
#include "output.h"
#include "dmatrix_algebra.h"

namespace RDK2 { namespace Geometry {

using namespace std;

double maxWeight(const AppliedVector*v, int size) {
	double mWeight = 0;
	for(int i=0;i<size;i++) {
		mWeight = std::max(mWeight, v[i].weight);
	}
	return mWeight;	
}

void output(const PPVec&vec, const char*filename) {
	ofstream ofs(filename);
	for(size_t a=0;a<vec.size();a++) {
		Point2d p = vec[a].getCartesian();
		ofs << p.x << " " << p.y << endl;
	}
}

void output(const std::string& file, const AVV& v, bool normalize, double scale) {
	output(file, &(v[0]), v.size(), normalize, scale);
}
void output(
	const std::string& file, const AppliedVector*v, int size, bool normalize, double scale)
{
	std::ofstream ofs(file.c_str());
	
	// il massimo e' messo a scale
	double mWeight = maxWeight(v,size);
	if(normalize) {
		scale /= mWeight;
		ofs << "# normalized, max weight=" << mWeight << std::endl;
	}
	for(int i=0;i<size;i++) {
		double w = v[i].weight * scale;
		Point2d from = v[i].where;
		Point2d to = v[i].where + Point2d::vers(v[i].theta) * w ;
		ofs << "# weight: " << w << " theta " <<v[i].theta << std::endl;
		ofs << from.x << " " << from.y << std::endl
			<<   to.x << " " <<   to.y << std::endl << std::endl;
	}
}

void output(const std::string& file, const OPVec& v, double scale) {
	std::ofstream ofs(file.c_str());
	for(OPVec::const_iterator i=v.begin();i!=v.end();++i) {
		Point2d from = (*i).point2();
		Point2d to = (*i).point2() + Point2d::vers(i->theta) * scale;
		ofs << from.x << " " << from.y << std::endl
			<<   to.x << " " <<   to.y << std::endl << std::endl;
	}
}

/////////////////////

void outputNormal(ostream&os, const AVV& v, bool norm, double s) {
	outputNormal(os, &(v[0]), v.size(), norm, s);
}

void outputNormal(const std::string& file, const AVV& v,bool norm, double s) {
	outputNormal(file, &(v[0]), v.size(), norm, s);
}

void outputNormal(const std::string& file, const AppliedVector*v, int size,
	bool norm, double s)
{
	std::ofstream ofs(file.c_str());
	outputNormal(ofs, v, size, norm, s);
}


void outputNormal(ostream&ofs, const AppliedVector*v, int size,
	bool norm, double s) {
		
	double max_weight = maxWeight(v,size);
	double scale = norm ? s/max_weight:s;
	
	ofs << "# List of oriented points with weight: " << std::endl;
	ofs << "# x y theta weight*scale theta2 weightNormalized" << std::endl;
	if(norm) 
	ofs << "# scale factor:  " << scale << std::endl;
	
	for(int i=0;i<size;i++) {
		ofs << v[i].where.x << " " << v[i].where.y << " " <<v[i].theta 
		<< " " << v[i].weight * scale
		<< " " << v[i].theta2 
		<< " " << v[i].weight / max_weight
		<< std::endl;
	}
}
//////////////////





void outputNormal(const std::string& file, const OPVec& v) {
	std::ofstream ofs(file.c_str());
	ofs << "# List of oriented points: " << std::endl;
	ofs << "# x y theta" << std::endl;
	for(OPVec::const_iterator i=v.begin();i!=v.end();++i)
		ofs << i->x << " " << i->y << " " << i->theta << std::endl;
}

void outputGaussian(std::ostream& ofs, Point2d estimate, DMatrix<double> variance) {
	Point2d evec[2];
	double  eval[2];
	if (eig22_gsl(variance, eval[0], evec[0], eval[1], evec[1]))
	{
		ofs << "# eig1 " << evec[0].toString() << " val " << eval[0] << endl;
		ofs << "# eig2 " << evec[1].toString() << " val " << eval[1] << endl;
		//double k = 2.41; // 70%
		// double k = 5.99; // 95%
		double k = 4; // 2 sigma
	
		int npoints = 64;
		Point2d points[npoints];
		for(int a=0;a<npoints;a++) {
			double theta = a * M_PI *2 / npoints;		
			Point2d cth = Point2d::vers(theta) * sqrt(k);
			Point2d ellipse =  
				evec[0] * sqrt(eval[0]) * (evec[0] * cth) +
				evec[1] * sqrt(eval[1]) * (evec[1] * cth);
			points[a] =  estimate + ellipse;
		}

		for(int a=0;a<=npoints;a++) {
			ofs << points[a%npoints].x << " " << points[a%npoints].y << endl;
		}
		ofs << endl << endl;
	}
}


}} // ns

