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

#include "laserdata.h"

#include <rdkcore/geometry/utils.h>
#include <rdkcore/rdkmath/rdkmath.h>

namespace RDK2 { namespace SensorData {
	using namespace RDK2::Geometry;
	using namespace std;
	
	DMatrixD LaserData::getFisherForXY() {
		DMatrixD fisher(2,2);
		
		fisher.setAll(0);
		for(size_t i=0;i< points.size();i++) {
			if( (!points[i].isValid()) || (!points[i].hasTrueAlpha()))  {
	//			cout << "ray i= " << i << " is invalid" << endl;
				continue;
			}
			double alpha_i = points[i].trueAlpha;
			double beta_i = alpha_i - (estimatedPose.theta+points[i].theta);
		//	cout << "ray i=" <<i << " valid = " << points[i].isValid() 
		//		<< " reading = " << points[i].reading << 
		//		 " alpha = " << alpha_i << " beta_i = " << beta_i  << endl; 
			
			fisher.el(0,0) += cos(alpha_i)*cos(alpha_i) / square(cos(beta_i));
			fisher.el(1,1) += sin(alpha_i)*sin(alpha_i) / square(cos(beta_i));
			fisher.el(0,1) += sin(alpha_i)*cos(alpha_i) / square(cos(beta_i));
			fisher.el(1,0) += sin(alpha_i)*cos(alpha_i) / square(cos(beta_i));
		}
		
		//cout << "fisher : " <<fisher<<endl;
		return fisher;
	}
	
	double LaserData::getFisherForTheta() {
		double fim = 0;
		for(int i=0;i<(int)points.size();i++) {
			if(!points[i].hasTrueAlpha()) continue;
			double r_i =  points[i].reading;
			double alpha_i = points[i].trueAlpha;
			double beta_i = alpha_i - (estimatedPose.theta+points[i].theta);

			fim += square(r_i) * square(tan(beta_i));  
		}
		return fim;
	}
	
	DMatrixD LaserData::getFisher() {
		// top right matrix of Fisher's information matrix
		DMatrixD tr(2,1);	
		
		tr.setAll(0);
		for(int i=0;i<(int)points.size();i++) {
			if(!points[i].hasTrueAlpha()) continue;
			
			double alpha_i = points[i].trueAlpha;
			double beta_i = alpha_i - (estimatedPose.theta+points[i].theta);
			
			tr.el(0,0) += cos(alpha_i) * tan(beta_i) / cos(beta_i);
			tr.el(1,0) += sin(alpha_i) * tan(beta_i) / cos(beta_i);
		}
		
		DMatrixD fim_xy = getFisherForXY();
		double fim_theta = getFisherForTheta();
		
		DMatrixD fim(3,3);
		
		fim.el(0,0) = fim_xy.el(0,0);
		fim.el(1,0) = fim_xy.el(1,0);
		fim.el(0,1) = fim_xy.el(0,1);
		fim.el(1,1) = fim_xy.el(1,1);

		fim.el(0,2) = fim.el(2,0) = tr.el(0,0);
		fim.el(1,2) = fim.el(2,1) = tr.el(1,0);
	
		fim.el(2,2) = fim_theta;
		
		return fim;
	}
}}
