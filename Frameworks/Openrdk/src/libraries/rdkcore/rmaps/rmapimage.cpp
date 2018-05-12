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

#include <sstream>
#include <cmath>
#include <cfloat>
#include <cstring>

#include <rdkcore/logging/logging.h>
#include <rdkcore/object/objectmanager.h>
#include <rdkcore/geometry/walk_line.h>
#define LOGGING_MODULE "RMapImage"

#include "rmapimage.h"

namespace RDK2 { namespace RMaps {

RDK2_FACTORY(RMapImage);

using namespace RDK2::Geometry;
using namespace std;

RMapImage::RMapImage() {
	image = new RImage(255,255,RImage::C8);
	x = y = theta = 0;
	realWidth = 10000; // mm
	//isPatch = false;
}

RMapImage::~RMapImage() {
	delete image;
}

std::string RMapImage::toString() const {
	Point2d topLeft,bottomRight;
	buf2world(0,0,topLeft.x, topLeft.y);
	buf2world(image->getWidth()-1,image->getHeight()-1,bottomRight.x, bottomRight.y);
	Point2d imax = Point2d::max(topLeft,bottomRight);
	Point2d imin = Point2d::min(topLeft,bottomRight);
	ostringstream oss;
	oss << "RImage{topleft: " << topLeft.toString() << " br " << 
		bottomRight.toString() << " Min-max: "
	<< imin.toString() << " - " << imax.toString() << "}";
	return oss.str();
}

RMapImage::RMapImage(double x, double y, double theta, double realWidth, RImage* image) {
	this->x = x;
	this->y = y;
	this->theta = theta;
	this->realWidth = realWidth;
	this->image = image;
	//this->isPatch = false;
}

RDK2::Object* RMapImage::clone() const {
	RMapImage * rmi = new RMapImage(x,y,theta,realWidth,new RImage(*image));
	//rmi->isPatch=isPatch;
	
	return rmi;
}

void RMapImage::setPixelW(double wx, double wy, unsigned char color) {
	int bx, by;
	if(world2buf(wx,wy,bx,by))
		//image->getPixels()[by][bx] = color;
		setPixelB(bx, by, color);
}

unsigned char RMapImage::getPixelW(double wx, double wy) const
{
	int bx, by;
	if (world2buf(wx, wy, bx, by)) return getPixelB(bx, by); //return image->getPixels()[by][bx];
	else return RImage::C8Obstacle;
}

double RMapImage::minWorldX() const { return x; }

double RMapImage::maxWorldX() const { return x + realWidth; }

double RMapImage::minWorldY() const { return y - realWidth / image->getWidth() * image->getHeight(); }

double RMapImage::maxWorldY() const { return y; }

bool RMapImage::world2buf(const RDK2::Geometry::Point2d& w, RDK2::Geometry::Point2i&b) const {
	return world2buf(w.x,w.y,b.x,b.y);
}

void RMapImage::buf2world(const RDK2::Geometry::Point2i& b, RDK2::Geometry::Point2d&w) const {
	buf2world(b.x,b.y,w.x,w.y);
}
		
bool RMapImage::world2buf(double wx, double wy, int& bx, int &by) const {
	bx =(int)( (wx - x)*(image->getWidth()/realWidth));
	by =(int)(-(wy - y)*(image->getWidth()/realWidth)); 
	return bx >= 0 && by >= 0 && bx < (int)image->getWidth() && by < (int)image->getHeight();
}

void RMapImage::buf2world(int bx, int by, double& wx, double& wy) const
{
	wx = (double) bx * realWidth / image->getWidth() + x;
	wy = (double) y - by * realWidth / image->getWidth();
}

double RMapImage::buf2world(int d) const
{
	// d is a distance, not a coordinate
	return (double) (d * realWidth / image->getWidth());
}

int RMapImage::world2buf(double d) const
{
	// d is a distance, not a coordinate
	return (int) (d * image->getWidth() / realWidth);
}

double RMapImage::calcClearance_w(double wx, double wy)
{
	int bx, by;
	if (!world2buf(wx, wy, bx, by)) return 0;

	return calcClearance_b(bx, by);
}

double RMapImage::calcClearance_b(int bx, int by)
{
    double min_distance2 = DBL_MAX;

	int r = 1;
	int startx = bx;
	int starty = by;
	int found_countdown = -1;
	int max_r = image->getWidth();
	if ((int)image->getHeight() < max_r) max_r = image->getHeight();

	while (found_countdown == -1 || found_countdown > 0) {
		// we start from the cell at the left of the last "circle"
		for (int d = 0; d < 4; d++) {
			int dx, dy, x, y;
			switch (d) {
				case 0: x = (int) startx - r; y = (int) starty; 	  dx = 1;  dy = -1; break;
				case 1: x = (int) startx; 	  y = (int) starty - r;   dx = 1;  dy = 1; break;
				case 2: x = (int) startx + r; y = (int) starty; 	  dx = -1; dy = 1; break;
				default: x = (int) startx;  	  y = (int) starty + r;   dx = -1; dy = -1; break;
			}
			for (int i = 0; i < r; i++) {
				if (y + i * dy < 0 || y + i * dy >= (int)image->getHeight()
				||  x + i * dx < 0 || x + i * dx >= (int)image->getWidth()
				//||	image->getPixels()[y + i * dy][x + i * dx] == C8Obstacle
				|| getPixelB(x+i*dx, y+i*dy) == C8Obstacle
				) {
					int ax = bx - (x + i * dx);
					int ay = by - (y + i * dy);
					double distance2 = ax * ax + ay * ay;
					if (distance2 < min_distance2) {
						// this is a better minimum distance
						min_distance2 = distance2;
						// if it's the first obstacle we find, we start the countdown
						if (found_countdown == -1) found_countdown = (int) ((double) r * 0.2929 + 1);
					}
				}
			}
		}
		if (found_countdown > 0) found_countdown--;
		r++;
		if (r > max_r) break;    // defensive programming
	}
	return sqrt(min_distance2) * realWidth / image->getWidth();
}

unsigned int RMapImage::markLine(
	// da 
	int from_x, int from_y, 
	// a
	int to_x, int to_y, 
	int width, int height,
	point*line, unsigned int maxLineLength) 
{
	unsigned int count = 0;
	int x0,y0,x1,y1;
	if( abs(to_x-from_x) > abs(to_y-from_y) ) {
		x0 = from_x; x1=to_x;
		y0 = from_y; y1=to_y;
		int inc = (to_x>from_x) ? 1:-1;
		for(int x=x0;x!=x1 && (count < maxLineLength);x+=inc) {
			if(x<0 || x>=width) continue;
			int y = y0 + (x-x0)*(y1-y0)/(x1-x0);
			if(y>=0 && y<height ){
				line[count].x = x;
				line[count].y = y;
				count ++;
			}	
		}
	} else {
		x0 = from_x; x1=to_x;
		y0 = from_y; y1=to_y;
		int inc = (to_y>from_y) ? 1:-1;
		for(int y=y0;y!=y1 && (count < maxLineLength);y+=inc) {
			if(y<0 || y>=height) continue;
			int x = x0 + (y-y0)*(x1-x0)/(y1-y0);
			if(x>=0 && x<width ) {
				line[count].x = x;
				line[count].y = y;
				count ++;
			}	
		}
	}
	
	return count;
}

void RMapImage::line(int x0, int y0, int x1, int y1, unsigned char color) {
	unsigned int max = 5000;
	point coords[5000];
	unsigned int ncoords = markLine(x0, y0, x1, y1, image->getWidth(), image->getHeight(),
	 	coords, max);
	//unsigned char** buf =(unsigned char**) image->getPixels();
	for(unsigned int a=0;a<ncoords;a++) {
		unsigned int x = coords[a].x;
		unsigned int y = coords[a].y;	
		setPixelB(x, y, color);	//buf[y][x]=color;
	}
}


/* F deve implementare
	bool shouldStop(x,y)
*/


template<class F>
bool markLine2(
	// da 
	int from_x, int from_y, 
	// a
	int to_x, int to_y, 
	int width, int height,
	F&f) 
{
	int x0,y0,x1,y1;
	if( abs(to_x-from_x) > abs(to_y-from_y) ) {
		x0 = from_x; x1=to_x;
		y0 = from_y; y1=to_y;
		int inc = (to_x>from_x) ? 1:-1;
		for(int x=x0;x!=x1;x+=inc) {
			if(x<0 || x>=width) continue;
			int y = y0 + (x-x0)*(y1-y0)/(x1-x0);
			if(y>=0 && y<height ){
				if(f.shouldStop(x,y))
					return true;
			}
		}
	} else {
		x0 = from_x; x1=to_x;
		y0 = from_y; y1=to_y;
		int inc = (to_y>from_y) ? 1:-1;
		for(int y=y0;y!=y1 ;y+=inc) {
			if(y<0 || y>=height) continue;
			int x = x0 + (y-y0)*(x1-x0)/(y1-y0);
			if(x>=0 && x<width ) {
				if(f.shouldStop(x,y))
					return true;
			}
		}
	}
	
	return false;
}


/*
enum RayTracingResult { 
			Obstacle, // abbiamo preso un ostacolo, ritorna distanza ostacolo
			Free,     // non abbiamo preso niente entro maxdist
			Unknown,  // abbiamo attraversato il blu, ritorna distanza libera
			Margin    // abbiamo sforato
}; */


struct Ant {
	unsigned char**buf;
	int x,y;
	
	bool shouldStop(int x,int y) {
		this -> x = x;
		this -> y = y;
	
		if(buf[y][x]!=C8Free) {
			return true;
		}
		
		return false;
	}

};

RMapImage::RayTracingResult RMapImage::rayTracing2(
	double /*px*/, double /*py*/, double /*ptheta*/, double /*maxdist*/, double& /*result*/) const
{
	RDK_ERROR_PRINTF("Function deprecated");
	return RMapImage::RayTracingResult(0);
#if 0
	double p2x = px + cos(ptheta) * maxdist * 1.1;
	double p2y = py + sin(ptheta) * maxdist * 1.1;
	
	int x0,x1,y0,y1;
	world2buf(px,   py, x0, y0);
	world2buf(p2x, p2y, x1, y1);

	//unsigned char ** buf =(unsigned char**) image->getPixels();
	Ant ant;
//	ant.buf = buf;
	ant.x = 0; ant.y = 0;
	
	bool found = 
		markLine2(x0, y0, x1, y1, 
			image->getWidth(), image->getHeight(),
	 		ant);
	
	if(found) {
		double wx, wy;
		buf2world(ant.x,ant.y,wx,wy);
		double dist = sqrt((wx-px)*(wx-px)+(wy-py)*(wy-py));
		result = dist;
		
		//if(buf[ant.y][ant.x] == C8Unknown)
		if (getPixelB(ant.x, ant.y) == C8Unknown)
			return Unknown;
			
		//if(buf[ant.y][ant.x] == C8Obstacle) {
		if (getPixelB(ant.x, ant.y) == C8Obstacle) {
			if(result<maxdist) {
				// ostacolo
				return Obstacle;
			}
			else {
				// tutto libero
				return Free;
			}
		}
		return Unknown;
	}
	
	double wx, wy;
	buf2world(ant.x,ant.y,wx,wy);
	double dist = sqrt((wx-px)*(wx-px)+(wy-py)*(wy-py));
	result = dist;
	return Margin;
#endif
}

void RMapImage::walkVars(double w1x, double w1y, double w2x, double w2y,
	int& p1x, int& p1y, int& p2x, int& p2y,
	unsigned int& steps, int& vx, int& vy,
	double& incx, double& incy)
{
	if (w1x < w2x || (w1x == w2x && w1y < w2y)) {
		world2buf(w1x, w1y, p1x, p1y);
		world2buf(w2x, w2y, p2x, p2y);
	}
	else {
		world2buf(w1x, w1y, p2x, p2y);
		world2buf(w2x, w2y, p1x, p1y);
	}

	vx = (p1x < p2x ? 1 : -1);
	vy = (p1y < p2y ? 1 : -1);
	unsigned int stepsx = abs(p1x - p2x);
	unsigned int stepsy = abs(p1y - p2y);
	steps = (stepsx > stepsy ? stepsx : stepsy);
	incx = (double) stepsx / steps;
	incy = (double) stepsy / steps;
}

RMapImage::RayTracingResult RMapImage::visibilityCheck(double w0x, double w0y, double w1x, double w1y, double* distancePtr)
{
/*	int x0, y0, x1,y1;
	world2buf(p0x, p0y, x0, y0);
	world2buf(p1x, p1y, x1, y1);*/

	
	int p0x, p0y, p1x, p1y, vx, vy;
	unsigned int steps;
	double incx, incy;
	walkVars(w0x, w0y, w1x, w1y, p0x, p0y, p1x, p1y, steps, vx, vy, incx, incy);
	
	if (!steps) return Free;

	double x = p0x, y = p0y;
	
	RayTracingResult currentResult = Free;
	for (unsigned int i = 0; i < steps; i++) {
		int ix = (int) x, iy = (int) y;
		if (ix < 0 || iy < 0 || ix >= (int)image->getWidth() || iy >= (int)image->getHeight()) return Obstacle;
		//else if (image->getPixels()[(int) y][(int) x] == C8Obstacle) {
		else if (getPixelB(x, y) == C8Obstacle) {
			if (distancePtr) *distancePtr = sqrt((p0x - x) * (p0x - x) + (p0y - y) * (p0y - y));
			return Obstacle;
		}
		//else if (image->getPixels()[(int) y][(int) x] == C8Unknown) {
		else if (getPixelB(x, y) == C8Unknown) {
			if (distancePtr) *distancePtr = sqrt((p0x - x) * (p0x - x) + (p0y - y) * (p0y - y));
			currentResult = Unknown;
		}
		x += incx * vx;
		y += incy * vy;
	}
	return currentResult;
}


void RMapImage::distMapRayTracing(const Point2d& start, const Point2d& end, bool& hitSomething, int min_dist_from_wall)
{
	Point2i startb, endb;
	world2buf(start.x, start.y, startb.x, startb.y);
	world2buf(end.x, end.y, endb.x, endb.y);
	LineWalk lw(startb, endb);
	
	do {
		Point2i p = lw.getPoint();
		if (p.x < 0 || p.y < 0 || p.x >= (int)image->getWidth() || p.y >= (int)image->getHeight()) {
			hitSomething = true;
			return;
		}
		
                 //if (image->getPixels()[p.y][p.x]< min_dist_from_wall) {
				 if (getPixelB(p.x, p.y) < min_dist_from_wall) {
			hitSomething = true;
			return;
		}

	} while (lw.next());
	
	hitSomething = false;
}

double RMapImage::obsDenRayTracing(const Point2d& start, const Point2d& end)
{
	Point2i startb, endb;
	world2buf(start.x, start.y, startb.x, startb.y);
	world2buf(end.x, end.y, endb.x, endb.y);
	LineWalk lw(startb, endb);
	double result=0;
	do {
		Point2i p = lw.getPoint();
		if (p.x < 0 || p.y < 0 || p.x >= (int)image->getWidth() || p.y >= (int)image->getHeight()) {
			continue;
		}
                double tmp;
		//if(image->getPixels()[p.x][p.y]==0)
		if (getPixelB(p.x, p.y) == 0)
                  tmp=2.0;
                else
                   //tmp=1.0/(double)(image->getPixels()[p.x][p.y]);
				   tmp = 1.0 / (double) (getPixelB(p.x, p.y));
             
                result+=tmp;

	} while (lw.next());
	
	return result;
}


void RMapImage::rayTracing(const Point2d& start, const Point2d& end, const RC8Set* obstacleColors,
	bool& hitSomething, RImage::C8Color& colorHit)
{
	Point2i startb, endb;
	world2buf(start.x, start.y, startb.x, startb.y);
	world2buf(end.x, end.y, endb.x, endb.y);
	LineWalk lw(startb, endb);
	
	do {
		Point2i p = lw.getPoint();
		if (p.x < 0 || p.y < 0 || p.x >= (int)image->getWidth() || p.y >= (int)image->getHeight()) {
			hitSomething = true;
			colorHit = C8Obstacle;
			return;
		}
		//if (obstacleColors->contains(image->getPixels()[p.y][p.x])) {
		if (obstacleColors->contains(getPixelB(p.x, p.y))) {
			hitSomething = true;
			//colorHit = (RImage::C8Color) image->getPixels()[p.y][p.x];
			colorHit = (RImage::C8Color) getPixelB(p.x, p.y);
			return;
		}
	} while (lw.next());
	
	hitSomething = false;
}

bool RMapImage::rayTracing(const Point2d& start, const Point2d& end, const RC8Set* obstacleColors,
	Point2d* collisionPoint, RImage::C8Color* colorHit)
{
	Point2i startb, endb;
	world2buf(start.x, start.y, startb.x, startb.y);
	world2buf(end.x, end.y, endb.x, endb.y);
	LineWalk lw(startb, endb);
	
	do {
		Point2i p = lw.getPoint();
		if (p.x < 0 || p.y < 0 || p.x >= (int)image->getWidth() || p.y >= (int)image->getHeight()) {
			if (colorHit) *colorHit = (RImage::C8Color) getPixelB(p.x, p.y);
			if (collisionPoint) buf2world(p, *collisionPoint);
			return true;
		}
		if (obstacleColors->contains(getPixelB(p.x, p.y))) {
			if (colorHit) *colorHit = (RImage::C8Color) getPixelB(p.x, p.y);
			if (collisionPoint) buf2world(p, *collisionPoint);
			return true;
		}
	} while (lw.next());
	if (collisionPoint) *collisionPoint = end;
	return false;
}

bool RMapImage::rayTracing(const Point2d& start, double direction, double maxDist, const RC8Set* obstacleColors,
	double* distance, RImage::C8Color* colorHit)
{
	Point2d end(start.x + cos(direction) * maxDist, start.y + sin(direction) * maxDist);
	Point2d p;
	bool b = rayTracing(start, end, obstacleColors, &p, colorHit);
	if (distance) *distance = start.distTo(p);
	return b;
}

void RMapImage::read(Reader* r) throw (ReadingException)
{
	r->startReading(getClassName());
		x = r->read_f32();
		y = r->read_f32();
		theta = r->read_f32();
		realWidth = r->read_f32();
		image = dynamic_cast<RImage*>(r->readObject());
	r->doneReading();
}

void RMapImage::write(Writer* w) const throw (WritingException)
{
	w->startWriting(getClassName());
		w->write_f32(x, "x");
		w->write_f32(y, "y");
		w->write_f32(theta, "theta");
		w->write_f32(realWidth, "realWidth");
		w->writeObject(true, image, "image");
	w->doneWriting();
}

void RMapImage::move(double xnew, double ynew, RImage * init){
	int xmin, ymin;
// 	RDK_DEBUG_PRINTF("xnew %.2f, ynew %.2f",xnew,ynew);
	world2buf(xnew,ynew,xmin,ymin);
	int xmax=xmin+image->getWidth(), ymax = ymin+image->getHeight();
	int xsize=std::min(image->getWidth(),(size_t)xmax)-std::max(0,xmin);
	int ysize=std::min(image->getHeight(),(size_t)ymax)-std::max(0,ymin);
	int startx=std::max(0,xmin), starty=std::max(0,ymin);
	int startx2=-std::min(0,xmin), starty2=-std::min(0,ymin);
// 	RDK_DEBUG_PRINTF("x:(%d,%d), y:(%d,%d), size:(%d,%d)",xmin,xmax,ymin,ymax,xsize,ysize);
// 	RDK_DEBUG_PRINTF("start:(%d,%d), start2:(%d,%d)",startx,starty,startx2,starty2);
	RImage *newImage = new RImage(*init);
	unsigned char * ibuf = image->getBuffer();
	unsigned char * buf = newImage->getBuffer();
	ibuf+=startx;
	ibuf+=starty*image->getWidth();
	buf+=startx2;
	buf+=starty2*newImage->getWidth();
 	for (int i=0;i<ysize;i++) {
		memcpy(buf,ibuf,xsize);
		ibuf+=image->getWidth();
		buf+=newImage->getWidth();
	}
	this->x=xnew;
	this->y=ynew;
	delete this->image;
	this->image=newImage;
	
}

void RMapImage::canvasReshape(double wd, double hg, double anch_x, double anch_y, unsigned char init_value)
{
	int anch_u, anch_v;
	world2buf(anch_x, anch_y, anch_u, anch_v);
	size_t imgw = ceil(wd / getMapDelta()), imgh = ceil(hg / getMapDelta());
   double neww = imgw * getMapDelta();
	image->canvasResize(imgw, imgh, anch_u, anch_v, init_value);
	this->realWidth = neww;
	buf2world(anch_u, anch_v, anch_x, anch_y);
	this->x = anch_x;
	this->y = anch_y;
}

bool RMapImage::centeredRobot(double rpx, double rpy,double * new_x, double* new_y, double off_size){
//  	RDK_DEBUG_PRINTF("sono centrato? rpx %f x %f rpy %f y %f realWidth %f realHeight %f ",rpx,x,rpy,y,realWidth,getRealHeight());

	if( (rpy-y)<-off_size && (rpy-y)>(-getRealHeight()+off_size) &&
			(rpx-x)>off_size && (rpx-x)<(realWidth-off_size))
		return true;

	*new_x=rpx-realWidth/2.0;
	*new_y=rpy+getRealHeight()/2.0;
//  	RDK_DEBUG_PRINTF("non ero centrato... adesso new_x %f neww_y %f",*new_x,*new_y); 
	return false;
}

}} // namespaces

#include "rmapimage_diffs.cpp"
