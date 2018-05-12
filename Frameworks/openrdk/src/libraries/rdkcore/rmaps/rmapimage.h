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

#ifndef RDK2_RMAPS_RMAPIMAGE
#define RDK2_RMAPS_RMAPIMAGE

#include <rdkcore/rgraphics/rimage.h>
#include <rdkcore/geometry/point.h>
#include <rdkcore/geometry/dmatrix.h>
#include <rdkcore/geometry/viewport.h>
#include <rdkcore/rgraphics/rc8set.h>

#define C8Obstacle RImage::C8Black
#define C8Free     RImage::C8White
#define C8Unknown  RImage::C8Blue
#define C8Path     RImage::C8Red
#define C8Stall    RImage::C8Magenta
#define C8Rough    RImage::C8Green
#define C8Glass    RImage::C8Cyan
#define C8StartObstacle RImage::C8Gray

namespace RDK2 { namespace RMaps {

using namespace RDK2::Meta;
using namespace RDK2::RGraphics;
using namespace Geometry;

/** An image with spatial placement. */
struct RMapImage: public RDK2::Object {
	public:
	/** Coordinates of (0,0) pixel */
	double x,y;
	
	/** Direction for segment (0,0)-(width,0) */
	double theta; 
	
	/** Width of image in mm (height is scaled accordingly) */
	double realWidth;
	
	/**
		The image data can be used to directly write on the map. You might 
		want to retrieve the data using image->getPixels()
	*/
	RImage* image;
	
	public:
		RMapImage();
		/** "image" is not cloned and it is now property of the RMapImage. */
		RMapImage(double x, double y, double theta, double realWidth, RImage*image);
		
		~RMapImage();
	
	public:
		void move(double x, double y, RImage* image);

		/**
		this is a function that reshape the map, setting appropriately the new 
		position and the new map dimensions. If the new canvas overlaps the old
		one, the overlapping area of the old map is copied unchanged in the new canvas.
		@param wd the new canvas (map) width
		@param hg the new canvas (map) height
		@param new_x the new x position of the (map) canvas
		@param new_y the new y position of the (map) canvas
		@param fillValue the value (at a byte level) used to fill the area of the new (map) canvas that
			does not ovelap the old (map) canvas.
		*/
		void canvasReshape(double wd, double hg, double new_x=0, double  new_y=0,
			unsigned char fillValue= (unsigned char)C8Unknown);

		/**
		this is a utility function that check whether or not the robot position
		is centered in the map. If not, it returns the new suggested position 
		for the new map
		@param robot_posx the x robot position to be tested
		@param robot_posy the y robot position to be tested
		@param new_x the new x map position
		@param new_y the new y map position
		@param off_size the robot is 'centered' if the distance 
				to the map border projected in
				one of its coordinate axes is greater than off_size. 
		*/
		bool centeredRobot(double robot_posx, double robot_posy, double * new_x, double* new_y, double off_size);
		
public:
		void read(Reader* r) throw (ReadingException);
		void write(Writer* w) const throw (WritingException);
		RDK2::Object* clone() const;

	public:
		bool knowsDiffs() { return true; }
		bool applyDiff(const ObjectDiff* diff);
		vector<ObjectDiff*> splitInDiffs(size_t maxSize);

		/// ritorna true se dentro
		bool world2buf(const RDK2::Geometry::Point2d& w, RDK2::Geometry::Point2i&b) const;
		void buf2world(const RDK2::Geometry::Point2i& b, RDK2::Geometry::Point2d&w) const;
		bool world2buf(double wx, double wy, int& x, int &y) const;
		void buf2world(int bx, int by, double& wx, double& wy) const;
		double buf2world(int d) const;
		int world2buf(double d) const;
		
		inline RDK2::Geometry::Point2d minWorld() const { return RDK2::Geometry::Point2d(minWorldX(), minWorldY()); }
		inline RDK2::Geometry::Point2d maxWorld() const { return RDK2::Geometry::Point2d(maxWorldX(), maxWorldY()); }
		double minWorldX() const;
		double maxWorldX() const;
		double minWorldY() const;
		double maxWorldY() const;
		
		inline double getRealWidth() const { return realWidth; }
		inline double getRealHeight() const { return realWidth * image->getHeight() / image->getWidth(); }
		inline double getMapDelta() const {return realWidth/image->getWidth(); }
		
		#define EPSILON 0.001
		inline bool sameGeometry(const RMapImage* mapImage) const {
			return fabs(x - mapImage->x) < EPSILON
				&& fabs(y - mapImage->y) < EPSILON
				&& fabs(theta - mapImage->theta) < EPSILON
				&& fabs(getRealWidth() - mapImage->getRealWidth()) < EPSILON
				&& image->getHeight() == mapImage->image->getHeight();
		}
		
		std::string toString() const;

	///
	/// Modifica immagini
	///
		void setPixelW(double wx, double wy, unsigned char color);
		unsigned char getPixelW(double wx, double wy) const;

		inline bool safeSetPixelB(size_t x, size_t y, unsigned char color) { return image->safeSetPixel(x, y, 0, color); }
		inline unsigned char safeGetPixelB(size_t x, size_t y, unsigned char defaultColor) const
		{ return image->safeGetPixel(x, y, 0, defaultColor); }
	
		inline void setPixelB(size_t x, size_t y, unsigned char color) { image->setPixel(x, y, 0, color); }
		inline unsigned char getPixelB(size_t x, size_t y) const { return image->getPixel(x, y, 0); }
		
		void line(int x0, int y0, int x1, int y1, unsigned char color);
	///
	/// Ray tracing e similari.
	///
		struct point { int x, y; }; // AC: FIXME: questo se lo becco...
		
		static unsigned int markLine(int x0, int y0, int x1, int y1,
			int width, int height,
			point * line, unsigned int maxLineLength);
		
		// obsolete
		enum RayTracingResult { 
			Obstacle=0, ///< abbiamo preso un ostacolo, ritorna distanza ostacolo
			Free=1,     ///< non abbiamo preso niente entro maxdist
			Unknown=2,  ///< abbiamo attraversato il blu, ritorna distanza libera
			Margin=3    ///< abbiamo sforato
		};
		
//		RayTracingResult rayTracing(double x, double y, double theta, double maxdist, double& result) const;

/* pose: world (mm,mm,rad) */
		void rayTracing(const Point2d& start, const Point2d& end, const RC8Set* obstacleColors,
			bool& hitSomething, RImage::C8Color& colorHit);
		//checks on a distance map if a path is too close to an obstacle (closer than mindist+1)
		void distMapRayTracing(const Point2d& start, const Point2d& end, bool& hitSomething, int min_dist);
                double obsDenRayTracing(const Point2d& start, const Point2d& end);	
		// obsolete
		RayTracingResult rayTracing2(double px, double py, double ptheta, double maxdist, double& result) const;
		// obsolete
		RayTracingResult visibilityCheck(double p0x, double p0y, double p1x, double p1y, double* distancePtr = 0);

		// remove all the rest and keep these two FIXME
		bool rayTracing(const Point2d& start, const Point2d& end, const RC8Set* obstacleColors,
			Point2d* collisionPoint = 0, RImage::C8Color* colorHit = 0);
		bool rayTracing(const Point2d& start, double direction, double maxDist, const RC8Set* obstacleColors,
			double* distance = 0, RImage::C8Color* colorHit = 0);
	
		double calcClearance_w(double wx, double wy);
		double calcClearance_b(int bx, int by);
	
		
	
	private:
		void walkVars(double w1x, double w1y, double w2x, double w2y,	// FIXME
		int& p1x, int& p1y, int& p2x, int& p2y,
		unsigned int& steps, int& vx, int& vy,
		double& incx, double& incy);
};

}} // namespaces

#endif
