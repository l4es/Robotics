
#ifndef  LANDMARK_INC
#define  LANDMARK_INC

#include <string>
#include <set>
#include <map>
#include <rdkcore/geometry/point.h>

/*
 * =====================================================================================
 *        Class:  landmark
 *  Description:  Class implementing a generic feature of the environment
 * =====================================================================================
 */
template < typename Point>
struct landmark: Point
{
	std::string tag;

	landmark (const std::string& t="unknown"): Point(),tag(t)
	{
	}

	landmark (const Point& p, const std::string& t): Point(p), tag(t)
	{
	}
}; /* ----------  end of template class landmark  ---------- */

typedef landmark<RDK2::Geometry::Point2d> Landmark2D;
typedef landmark<RDK2::Geometry::Point3d> Landmark3D;

/*
 * =====================================================================================
 *        Class:  LandmarkClass
 *  Description:  Class defining a class of Landmark
 * =====================================================================================
 */
template<typename LandmarkType>
struct LandmarkClass: std::pair<std::string, std::set<LandmarkType> >
{
	typedef std::string tag;
	typedef std::set<LandmarkType> landmarks;
}; /* ----------  end of template class LandmarkClass  ---------- */

typedef LandmarkClass<Landmark2D> LandmarkClass2D;
typedef LandmarkClass<Landmark3D> LandmarkClass3D;

typedef std::map<LandmarkClass2D::tag, LandmarkClass2D::landmarks> Landmarks2DMap;
typedef std::map<LandmarkClass3D::tag, LandmarkClass3D::landmarks> Landmarks3DMap;

#endif   /* ----- #ifndef LANDMARK_INC  ----- */
