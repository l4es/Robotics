
/**
* @file camerainfo.h
*
* Declaration of class CameraInfo
*/

#ifndef __camerainfo_h_
#define __camerainfo_h_

#include <rdkcore/geometry/point.h>

#include <string>
#include <sstream>

struct CameraParams
{
	CameraParams():
		resolutionWidth(0),
		resolutionHeight(0),
		focalLength(1),
		focalLengthInv(1),
		focalLenPow2(1),
		openingAngle(),
		opticalCenter()
	{}
	
	/** Intrinsic camera parameters: axis skew is modelled as 0 (90� perfectly orthogonal XY)
	 * and the same has been modeled for focal axis aspect ratio; distortion is considering
	 * only 2nd and 4th order coefficients of radial model, which account for about 95% of total.
	 */

	int resolutionWidth;
	int resolutionHeight;

	double focalLength;
	double focalLengthInv; // (1/focalLength) used to speed up certain calculations
	double focalLenPow2;

	RDK2::Geometry::Point2d openingAngle;
	RDK2::Geometry::Point2d opticalCenter;
};

class CameraInfo: public CameraParams
{

public :

	CameraInfo(std::string t="undefined"):
		CameraParams(),
		type(t)
	{}

	virtual std::string toString() const
	{
		std::ostringstream oss;
		oss << "(" << " resolution:" << this->resolutionWidth << ","  << " "  << this->resolutionHeight << "\n"
		<< " FocalLenght : " << this->focalLength << ", OpticalCenter : " << this->opticalCenter << "\n OpeningAngle " << this->openingAngle << ")";
		return oss.str();
	}

	virtual std::string getType() const { return type; };

	protected:
		std::string type;

};


#endif
