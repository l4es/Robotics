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

#include <string>
#include <set>

#include <rdkcore/textutils/textutils.h>

#include "dmatrix.h"
#include "utils.h"
#include "point.h"
#include "segment.h"
#include "geometryfunctions.h"

using namespace RDK2::TextUtils;
using namespace std;

namespace RDK2 { namespace Geometry {

Point2d projectPointOntoLineOrSegment(const Point2d& p, const Segment2d& segm, bool itIsALine)
{
	double vx = segm.p1.x - segm.p0.x, vy = segm.p1.y - segm.p0.y;
	double wx = p.x - segm.p0.x, wy = p.y - segm.p0.y;
	double c1 = (wx*vx + wy*vy);
	if (!itIsALine && c1 <= 0) return segm.p0;
	else {
		double c2 = (vx*vx + vy*vy);
		if (!itIsALine && c2 <= c1) return segm.p1;
		double b = c1 / c2;
		double bx = segm.p0.x + b*vx;
		double by = segm.p0.y + b*vy;
		return Point2d(bx, by);
	}
}

Point2d projectPointOntoLine(const Point2d& p, const Segment2d& segm)
{
	return projectPointOntoLineOrSegment(p, segm, true);
}

Point2d projectPointOntoSegment(const Point2d& p, const Segment2d& segm)
{
	return projectPointOntoLineOrSegment(p, segm, false);
}

double pointDistToLine(const Point2d& p, const Segment2d& segm)
{
	Point2d projectedPoint = projectPointOntoLine(p, segm);
	return p.distTo(projectedPoint);
}

double pointDistToSegment(const Point2d& p, const Segment2d& segm)
{
	Point2d projectedPoint = projectPointOntoSegment(p, segm);
	return p.distTo(projectedPoint);
}

vector<Segment2d> recursiveLineFitting(const vector<Point2d>& points, double maxErrorAllowed)
{
	set<int> endPoints;
	endPoints.insert(0);
	endPoints.insert(points.size() - 1);
	double maxDist = 0;
	do {
		int maxDistIndex = -1;
		maxDist = 0;
		set<int>::iterator it = endPoints.begin();
		set<int>::iterator it2 = ++endPoints.begin();
		for (; it2 != endPoints.end(); ++it, ++it2) {
			int i0 = *it;
			int i1 = *it2;
			Segment2d s;
			s.p0 = points[i0];
			s.p1 = points[i1];
			for (int j = i0 + 1; j < i1; j++) {
				double dist = pointDistFromLine(points[j], s);
				if (dist > maxDist) {
					maxDist = dist;
					maxDistIndex = j;
				}
			}
		}
		if (maxDist > maxErrorAllowed) {
			endPoints.insert(maxDistIndex);
		}
	} while (maxDist > maxErrorAllowed);

	vector<Segment2d> retval;
	set<int>::iterator it = endPoints.begin();
	set<int>::iterator it2 = ++endPoints.begin();
	for (; it2 != endPoints.end(); ++it, ++it2) {
		Segment2d s;
		s.p0 = points[*it];
		s.p1 = points[*it2];
		retval.push_back(s);
	}
	return retval;
}

//  public domain function by Darel Rex Finley, 2006
//  Determines the intersection point of the line segment defined by points A and B
//  with the line segment defined by points C and D.
//
//  Returns YES if the intersection point was found, and stores that point in X,Y.
//  Returns NO if there is no determinable intersection point, in which case X,Y will
//  be unmodified.
bool segmentIntersection(const Segment2d& segm1, const Segment2d& segm2, Point2d* intersection)
{
	double Ax = segm1.p0.x;	double Ay = segm1.p0.y;
	double Bx = segm1.p1.x;	double By = segm1.p1.y;
	double Cx = segm2.p0.x;	double Cy = segm2.p0.y;
	double Dx = segm2.p1.x;	double Dy = segm2.p1.y;

  double  distAB, theCos, theSin, newX, ABpos ;

  //  Fail if either line segment is zero-length.
  if ((Ax==Bx && Ay==By) || (Cx==Dx && Cy==Dy)) return false;

  //  Fail if the segments share an end-point.
  if ((Ax==Cx && Ay==Cy) || (Bx==Cx && By==Cy)
  ||  (Ax==Dx && Ay==Dy) || (Bx==Dx && By==Dy)) {
    return false; }

  //  (1) Translate the system so that point A is on the origin.
  Bx-=Ax; By-=Ay;
  Cx-=Ax; Cy-=Ay;
  Dx-=Ax; Dy-=Ay;

  //  Discover the length of segment A-B.
  distAB=sqrt(Bx*Bx+By*By);

  //  (2) Rotate the system so that point B is on the positive X axis.
  theCos=Bx/distAB;
  theSin=By/distAB;
  newX=Cx*theCos+Cy*theSin;
  Cy  =Cy*theCos-Cx*theSin; Cx=newX;
  newX=Dx*theCos+Dy*theSin;
  Dy  =Dy*theCos-Dx*theSin; Dx=newX;

  //  Fail if segment C-D doesn't cross line A-B.
  if ((Cy<0. && Dy<0.) || (Cy>=0. && Dy>=0.)) return false;

  //  (3) Discover the position of the intersection point along line A-B.
  ABpos=Dx+(Cx-Dx)*Dy/(Dy-Cy);

  //  Fail if segment C-D crosses line A-B outside of segment A-B.
  if (ABpos<0. || ABpos>distAB) return false;

  //  (4) Apply the discovered position to line A-B in the original coordinate system.
  if (intersection) {
	  intersection->x=Ax+ABpos*theCos;
	  intersection->y=Ay+ABpos*theSin;
  }

  //  Success.
  return true;
}

//  public domain function by Darel Rex Finley, 2006



//  Determines the intersection point of the line defined by points A and B with the
//  line defined by points C and D.
//
//  Returns true if the intersection point was found, and stores that point in X,Y.
//  Returns false if there is no determinable intersection point, in which case X,Y will
//  be unmodified.

bool lineIntersection(const Segment2d& line1, const Segment2d& line2, Point2d* intersection)
{
	double Ax = line1.p0.x;	double Ay = line1.p0.y;
	double Bx = line1.p1.x;	double By = line1.p1.y;
	double Cx = line2.p0.x;	double Cy = line2.p0.y;
	double Dx = line2.p1.x;	double Dy = line2.p1.y;

  double  distAB, theCos, theSin, newX, ABpos ;

  //  Fail if either line is undefined.
  if ((Ax==Bx && Ay==By) || (Cx==Dx && Cy==Dy)) return false;

  //  (1) Translate the system so that point A is on the origin.
  Bx-=Ax; By-=Ay;
  Cx-=Ax; Cy-=Ay;
  Dx-=Ax; Dy-=Ay;

  //  Discover the length of segment A-B.
  distAB=sqrt(Bx*Bx+By*By);

  //  (2) Rotate the system so that point B is on the positive X axis.
  theCos=Bx/distAB;
  theSin=By/distAB;
  newX=Cx*theCos+Cy*theSin;
  Cy  =Cy*theCos-Cx*theSin; Cx=newX;
  newX=Dx*theCos+Dy*theSin;
  Dy  =Dy*theCos-Dx*theSin; Dx=newX;

  //  Fail if the lines are parallel.
  if (Cy==Dy) return false;

  //  (3) Discover the position of the intersection point along line A-B.
  ABpos=Dx+(Cx-Dx)*Dy/(Dy-Cy);

  //  (4) Apply the discovered position to line A-B in the original coordinate system.
  if (intersection) {
	  intersection->x=Ax+ABpos*theCos;
	  intersection->y=Ay+ABpos*theSin;
  }

  //  Success.
  return true; 
}

}} // ns
