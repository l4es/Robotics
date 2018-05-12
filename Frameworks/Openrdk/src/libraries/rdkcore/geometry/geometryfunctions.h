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

#ifndef RDK2_GEOMETRY_GEOMETRYFUNCTIONS
#define RDK2_GEOMETRY_GEOMETRYFUNCTIONS

#include <cmath>
#include <vector>

#include "point.h"
#include "segment.h"

namespace RDK2 { namespace Geometry {

using namespace std;
	
Point2d projectPointOntoLine(const Point2d& p, const Segment2d& segm);
Point2d projectPointOntoSegment(const Point2d& p, const Segment2d& segm);
// NEW functions
double pointDistToLine(const Point2d& p, const Segment2d& segm);
double pointDistToSegment(const Point2d& p, const Segment2d& segm);
// DEPRECATED aliases
inline double pointDistFromLine(const Point2d& p, const Segment2d& segm) { return pointDistToLine(p, segm); }
inline double pointDistFromSegment(const Point2d& p, const Segment2d& segm) { return pointDistToSegment(p, segm); }

bool segmentIntersection(const Segment2d& segm1, const Segment2d& segm2, Point2d* intersection);
bool lineIntersection(const Segment2d& line1, const Segment2d& line2, Point2d* intersection);

vector<Segment2d> recursiveLineFitting(const vector<Point2d>& points, double maxErrorAllowed);

inline Point2d toLocalReferenceSystem(const Point2d& point, const Point2od& localReferenceSystem)
{
	Point2d tempp(point.x - localReferenceSystem.x, point.y - localReferenceSystem.y);
	tempp = tempp.rot(-localReferenceSystem.theta);
	return tempp;
}

inline Point2od toLocalReferenceSystem(const Point2od& point, const Point2od& localReferenceSystem)
{
	Point2d tempp(point.x - localReferenceSystem.x, point.y - localReferenceSystem.y);
	tempp = tempp.rot(-localReferenceSystem.theta);
	return Point2od(tempp.x, tempp.y, point.theta - localReferenceSystem.theta);
}

inline Point2d toGlobalReferenceSystem(const Point2d& point, const Point2od& localReferenceSystem)
{
	Point2d tempp(point.x, point.y);
	tempp = tempp.rot(localReferenceSystem.theta);
	tempp.x = tempp.x + localReferenceSystem.x;
	tempp.y = tempp.y + localReferenceSystem.y;
	return tempp;
}

inline Point2od toGlobalReferenceSystem(const Point2od& point, const Point2od& localReferenceSystem)
{
	Point2d tempp(point.x, point.y);
	tempp = tempp.rot(localReferenceSystem.theta);
	tempp.x = tempp.x + localReferenceSystem.x;
	tempp.y = tempp.y + localReferenceSystem.y;
	return Point2od(tempp.x, tempp.y, point.theta + localReferenceSystem.theta);
}

}} // ns

#endif
