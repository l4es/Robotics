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

#ifndef RDK2_GEOMETRY_REFERENCE
#define RDK2_GEOMETRY_REFERENCE

#include "point.h"
#include "otherpoints.h"
#include <vector>

namespace RDK2 { namespace Geometry {

	/** 
	 *  Metodi per il cambio del sistema di riferimento.
	 *
	 *  I metodi changeRef() si dovevano chiamare world2local 
	 */
	
		/** Ritorna p visto da newReference */
		Point2d changeRef(Point2od newReference, Point2d p);

		/** Ritorna p visto da newReference */
		PolarPoint changeRef(Point2od newReference, PolarPoint p);

		/** Ritorna p visto da newReference */
		AppliedVector changeRef(Point2od newReference, AppliedVector p);
		
		template<class X>
		std::vector<X> changeRef(Point2od newReference, const std::vector<X>& vec ) {
			std::vector<X> temp;
			for(size_t a=0;a<vec.size();a++) 
				temp.push_back(changeRef(newReference, vec[a]));
			return temp;
		}

		template<class X>
		void changeRef(Point2od newReference, const std::vector<X>& vec, std::vector<X> &out ) {
			for(size_t a=0;a<vec.size();a++) 
				out.push_back(changeRef(newReference, vec[a]));
		}

		/** p � locale a ref, ritorna p in coordinate mondo */
		AppliedVector local2world(Point2od ref, AppliedVector p);
		
		/** p � locale a ref, ritorna p in coordinate mondo */
		Point2od local2world(Point2od ref, Point2od p);

		template<class X>
		void local2world(Point2od ref, const std::vector<X>& vec, std::vector<X> &out ) {
			for(size_t a=0;a<vec.size();a++) 
				out.push_back(local2world(ref, vec[a]));
		}
	
	// Ruota di ds.theta e trasla di ds.x,ds.y
	PPVec translateTo(const PPVec&vec, Point2od ds);
	PolarPoint translateTo(PolarPoint p, Point2od ds);
	
}} // ns

#endif
