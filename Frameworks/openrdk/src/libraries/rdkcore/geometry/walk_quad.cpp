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

#include "point.h"
#include "walk_line.h"
#include "walk_quad.h"

namespace RDK2 { namespace Geometry {

void QuadWalk::costrTri(Point2i &p1, Point2i &p2, Point2i &p3)
{
	//The three points are on the same line
	if ((int)fabs((float)(p3.x*(p1.y-p2.y)+p3.y*(p2.x-p1.x)+p2.y*p1.x-p1.y*p2.x)/
		      (float)(hypot(p1.y-p2.y,p2.x-p1.x)) )<3  )   //  3 pixels
	{
		int bl=p1.x; int tr=p1.x; 
		Point2i a=p1;
		Point2i b=p1;
		if (p2.x<=bl) {bl=p2.x; a=p2;}
		if (p2.x>=tr) {tr=p2.x; b=p2;}
		if (p3.x<=bl) {bl=p3.x; a=p3;}
		if (p3.x>=tr) {tr=p3.x; b=p3;}
		if ((p1.x==p2.x) && (p2.x==p3.x)) 
		{
			int mm=std::min(p1.y,std::min(p2.y,p3.y)); 
			int MM=std::max(p1.y,std::max(p2.y,p3.y));
			Point2i a(p1.x,mm); 
			Point2i b(p1.x,MM);
			ptr=new LineWalk(a,b);
		} else ptr=new LineWalk(a,b);
		triangle=false;
		line=true;
		return;
	}

	//The three points form a triangle
	minI=std::min(p1.y,std::min(p2.y,p3.y));
	minJ=std::min(p1.x,std::min(p2.x,p3.x));
	maxI=std::max(p1.y,std::max(p2.y,p3.y));
	maxJ=std::max(p1.x,std::max(p2.x,p3.x));

	a1=(double)(p3.y-p1.y);   //retta per AC
	b1=(double)(p3.x-p1.x);
	p1x=p1.x; p1y=p1.y;
	if (a1<0) {a1=-a1; b1=-b1; p1x=p3.x; p1y=p3.y;}
	if ((a1==0) && (b1<0)) {a1=-a1; b1=-b1; p1x=p3.x; p1y=p3.y;}

	a2=(double)(p2.y-p1.y);   //retta per AB
	b2=(double)(p2.x-p1.x);
	p2x=p1.x; p2y=p1.y;
	if (a2<0) {a2=-a2; b2=-b2; p2x=p2.x; p2y=p2.y;}
	if ((a2==0) && (b2<0)) {a2=-a2; b2=-b2; p2x=p2.x; p2y=p2.y;}

	a3=(double)(p3.y-p2.y);   //retta per CB
	b3=(double)(p3.x-p2.x);
	p3x=p2.x; p3y=p2.y;
	if (a3<0) {a3=-a3; b3=-b3; p3x=p3.x; p3y=p3.y;}
	if ((a3==0) && (b3<0)) {a3=-a3; b3=-b3; p3x=p3.x; p3y=p3.y;}
	
	rewind();
}

QuadWalk::QuadWalk(Point2i p1, Point2i p2, Point2i p3, Point2i p4) {
// Canonical raster graphic requires the rule ''a polygon contains its bottom and left border but not its top and right one''
	quad=true;
	triangle=false;
	line=false;
	
	if ( ((p1.x==p2.x) && (p1.y==p2.y)) ||
	     ((p1.x==p3.x) && (p1.y==p3.y)) ||
	     ((p1.x==p4.x) && (p1.y==p4.y)) 
	     )   {
		quad=false;
		triangle=true; 
		costrTri(p2,p3,p4);
		return;
		}

	minI=std::min(p1.y,std::min(p2.y,std::min(p3.y,p4.y)));
	minJ=std::min(p1.x,std::min(p2.x,std::min(p3.x,p4.x)));
	maxI=std::max(p1.y,std::max(p2.y,std::max(p3.y,p4.y)));
	maxJ=std::max(p1.x,std::max(p2.x,std::max(p3.x,p4.x)));
	
	//retrieve the four contour lines
	
	float AC=hypot(p3.x-p1.x,p3.y-p1.y);

	double a=-1.*(double)(p2.y-p1.y);
	double b=(double)(p2.x-p1.x);
	double sqroot=hypot(a,b);
	double c=((double)(p2.y*p1.x-p1.y*p2.x))/sqroot;
	a=a/sqroot; b=b/sqroot;
	float h=fabs(a*p3.x+b*p3.y+c);
	float ang1=(asin(h/AC))*180./M_PI;	//angolo CAB
	if ((b*(p3.x-p1.x)-a*(p3.y-p1.y))<0)
	{ ang1=180.-ang1; //printf("\nscambio!");
	}

	a=-1.*(double)(p4.y-p1.y);
	b=(double)(p4.x-p1.x);
	sqroot=hypot(a,b);
	c=((double)(p4.y*p1.x-p1.y*p4.x))/sqroot;
	a=a/sqroot; b=b/sqroot;
	h=fabs(a*p3.x+b*p3.y+c);
	float ang2=(asin(h/AC))*180./M_PI;     //angolo CAD
	if ((b*(p3.x-p1.x)-a*(p3.y-p1.y))<0)
	{ang2=180.-ang2; //printf("\nscambio!");
	}

	float AB=hypot(p2.x-p1.x,p2.y-p1.y);
	h=fabs(a*p2.x+b*p2.y+c);
	float ang3=(asin(h/AB))*180./M_PI;     //angolo BAD
	if ((b*(p2.x-p1.x)-a*(p2.y-p1.y))<0)
	{ang3=180.-ang3; //printf("\nscambio!");
	}

	if ((ang2>ang1) && (ang2>ang3))
	{
		//printf("\nL'angolo CAD e' il maggiore: CAB=%3.1f, CAD=%3.1f, BAD=%3.1f",ang1,ang2,ang3);
		a1=(double)(p4.y-p1.y);   //retta per AD
		b1=(double)(p4.x-p1.x);
		p1x=p1.x; p1y=p1.y;
		if (a1<0) {a1=-a1; b1=-b1; p1x=p4.x; p1y=p4.y;}
		if ((a1==0) && (b1<0)) {a1=-a1; b1=-b1; p1x=p4.x; p1y=p4.y;}

		a2=(double)(p3.y-p1.y);   //retta per AC
		b2=(double)(p3.x-p1.x);
		p2x=p1.x; p2y=p1.y;
		if (a2<0) {a2=-a2; b2=-b2; p2x=p3.x; p2y=p3.y;}
		if ((a2==0) && (b2<0)) {a2=-a2; b2=-b2; p2x=p3.x; p2y=p3.y;}

		a3=(double)(p3.y-p2.y);   //retta per CB
		b3=(double)(p3.x-p2.x);
		p3x=p2.x; p3y=p2.y;
		if (a3<0) {a3=-a3; b3=-b3; p3x=p3.x; p3y=p3.y;}
		if ((a3==0) && (b3<0)) {a3=-a3; b3=-b3; p3x=p3.x; p3y=p3.y;}

		a4=(double)(p4.y-p2.y);   //retta per BD
		b4=(double)(p4.x-p2.x);
		p4x=p2.x; p4y=p2.y;
		if (a4<0) {a4=-a4; b4=-b4; p4x=p4.x; p4y=p4.y;}
		if ((a4==0) && (b4<0)) {a4=-a4; b4=-b4; p4x=p4.x; p4y=p4.y;}
	} else

	if ((ang3>ang2) && (ang3>ang1))
	{
		//printf("\nL'angolo BAD e' il maggiore: CAB=%3.1f, CAD=%3.1f, BAD=%3.1f",ang1,ang2,ang3);
		a1=(double)(p4.y-p1.y);   //retta per AD
		b1=(double)(p4.x-p1.x);
		p1x=p1.x; p1y=p1.y;
		if (a1<0) {a1=-a1; b1=-b1; p1x=p4.x; p1y=p4.y;}
		if ((a1==0) && (b1<0)) {a1=-a1; b1=-b1; p1x=p4.x; p1y=p4.y;}

		a2=(double)(p2.y-p1.y);   //retta per AB
		b2=(double)(p2.x-p1.x);
		p2x=p1.x; p2y=p1.y;
		if (a2<0) {a2=-a2; b2=-b2; p2x=p2.x; p2y=p2.y;}
		if ((a2==0) && (b2<0)) {a2=-a2; b2=-b2; p2x=p2.x; p2y=p2.y;}

		a3=(double)(p3.y-p4.y);   //retta per CD
		b3=(double)(p3.x-p4.x);
		p3x=p4.x;p3y=p4.y;
		if (a3<0) {a3=-a3; b3=-b3; p3x=p3.x; p3y=p3.y;}
		if ((a3==0) && (b3<0)) {a3=-a3; b3=-b3; p3x=p3.x; p3y=p3.y;}

		a4=(double)(p3.y-p2.y);   //retta per BC
		b4=(double)(p3.x-p2.x);
		p4x=p2.x;p4y=p2.y;
		if (a4<0) {a4=-a4; b4=-b4; p4x=p3.x; p4y=p3.y;}
		if ((a4==0) && (b4<0)) {a4=-a4; b4=-b4; p4x=p3.x; p4y=p3.y;}
	} else

	if ((ang1>ang2) && (ang1>ang3))
	{
		//printf("\nL'angolo CAB e' il maggiore: CAB=%3.1f, CAD=%3.1f, BAD=%3.1f",ang1,ang2,ang3);
		a1=(double)(p2.y-p1.y);   //retta per AB
		b1=(double)(p2.x-p1.x);
		p1x=p1.x; p1y=p1.y;
		if (a1<0) {a1=-a1; b1=-b1; p1x=p2.x; p1y=p2.y;}
		if ((a1==0) && (b1<0)) {a1=-a1; b1=-b1; p1x=p2.x; p1y=p2.y;}

		a2=(double)(p3.y-p1.y);   //retta per AC
		b2=(double)(p3.x-p1.x);
		p2x=p1.x; p2y=p1.y;
		if (a2<0) {a2=-a2; b2=-b2; p2x=p3.x; p2y=p3.y;}
		if ((a2==0) && (b2<0)) {a2=-a2; b2=-b2; p2x=p3.x; p2y=p3.y;}

		a3=(double)(p3.y-p4.y);   //retta per CD
		b3=(double)(p3.x-p4.x);
		p3x=p4.x; p3y=p4.y;
		if (a3<0) {a3=-a3; b3=-b3; p3x=p3.x; p3y=p3.y;}
		if ((a3==0) && (b3<0)) {a3=-a3; b3=-b3; p3x=p3.x; p3y=p3.y;}

		a4=(double)(p4.y-p2.y);   //retta per BD
		b4=(double)(p4.x-p2.x);
		p4x=p2.x; p4y=p2.y;
		if (a4<0) {a4=-a4; b4=-b4; p4x=p4.x; p4y=p4.y;}
		if ((a4==0) && (b4<0)) {a4=-a4; b4=-b4; p4x=p4.x; p4y=p4.y;}
	}

	else {
		quad=false;
		triangle=true;
		if (ang1==ang2) 
		{
			if (hypot(p2.x-p1.x,p2.y-p1.y)>hypot(p4.x-p1.x,p4.y-p1.y)) costrTri(p1,p2,p3);
			else costrTri(p1,p3,p4);
		} else if(ang2==ang3) {
		if (hypot(p2.x-p1.x,p2.y-p1.y)>hypot(p3.x-p1.x,p3.y-p1.y)) costrTri(p1,p2,p4);
			else costrTri(p1,p3,p4);
		} else //ang1==ang3
			if (hypot(p3.x-p1.x,p3.y-p1.y)>hypot(p4.x-p1.x,p4.y-p1.y)) costrTri(p1,p2,p3);
			else costrTri(p1,p2,p4);
		return;
	}
	//end retrieving the four contour lines
		
	rewind();
} //costruttore

bool QuadWalk::appartiene() {
	if ( (j>=j1) && (!sup1) && (t1>=0.) && (t1<1.) )
	{
		conta++;
		sup1=true;
	}
	if ((j>=j2) && (!sup2) && (t2>=0.) && (t2<1.) )
	{
		conta++;
		sup2=true;
	}
	if ((j>=j3) && (!sup3) && (t3>=0.) && (t3<1.) )
	{
		conta++;
		sup3=true;
	}
	if ((quad) && (j>=j4) && (!sup4) && (t4>=0.) && (t4<1.) )
	{
		conta++;
		sup4=true;
	}
	if (conta%2!=0)
		return true; else return false;
}

bool QuadWalk::next() {
	if (line) return ptr->next();
	while (i<maxI) 
	{
		while(j<maxJ)
		{
			j++;
			if (appartiene()) return true;
		}
		i++;
		j=minJ-1;
		conta=0;
		sup1=false;sup2=false;sup3=false;sup4=false;
		//j1=(-c1-b1*i)/a1; j2=(-c2-b2*i)/a2; j3=(-c3-b3*i)/a3; j4=(-c4-b4*i)/a4;
		t1=(i-p1y)/a1; t2=(i-p2y)/a2; t3=(i-p3y)/a3; 
		if (quad) t4=(i-p4y)/a4;
		j1=(int)(p1x+b1*t1); j2=(int)(p2x+b2*t2); j3=(int)(p3x+b3*t3); 
		if (quad) j4=(int)(p4x+b4*t4);
	}
	return false;
}

void QuadWalk::rewind()
{
	if (line) {ptr->rewind(); return;}
	
	i=minI;
	j=minJ;
	conta=0;
	sup1=false;sup2=false;sup3=false;sup4=false;
	//j1=(-c1-b1*i)/a1; j2=(-c2-b2*i)/a2; j3=(-c3-b3*i)/a3; j4=(-c4-b4*i)/a4;
	t1=(i-p1y)/a1; t2=(i-p2y)/a2; t3=(i-p3y)/a3; 
	if (quad) t4=(i-p4y)/a4;
	j1=(int)(p1x+b1*t1); j2=(int)(p2x+b2*t2); j3=(int)(p3x+b3*t3); 
	if (quad) j4=(int)(p4x+b4*t4);
	if (!appartiene()) next();
}

Point2i QuadWalk::getPoint() const {
	if (!line) return Point2i(j, i);
	else return ptr->getPoint();
}


}} // ns
