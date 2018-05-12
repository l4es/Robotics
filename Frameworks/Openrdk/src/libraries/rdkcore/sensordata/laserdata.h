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

#ifndef H_SENSORDATA_LASERDATA
#define H_SENSORDATA_LASERDATA

#include <rdkcore/geometry/point.h>
#include <rdkcore/geometry/otherpoints.h>
#include <rdkcore/geometry/dmatrix.h>
#include <rdkcore/geometry/otherpoints.h>

#include "sensordata.h"
#include "laserparser.h"

namespace RDK2 { namespace SensorData {

/** @note Laser data points (beams) are expected to be equally distantiated.
 */
	
struct LaserData: public BaseSensorData {

	/// Position of laser relative to robot. 
	RDK2::Geometry::Point3od laserPose;
	
	/// Minimum and maximum sensor range. Points above and below are invalid. 
	double minReading, maxReading;
	
	/// Span of data. Probably minTheta=-PI/2, maxTheta=+PI/2 
	double minTheta, maxTheta;
	
	/// Roll angle of robot position
	double roll;

	/// Pitch angle of robot position
	double pitch;
	
	double laserPan;
	
	struct LaserPoint {
		/// This is redundant but good to have
		double theta;
		
		/// Invalid rays have reading = 0; however this might change
		/// and you should check for validity with isValid().
		double reading;
		double intensity;
		
		/// Direction of the normal to the surface (radians) in
		/// world frame of reference.
		/// This value is not saved when serializing.
		double trueAlpha;
		
		/// Constructor
		LaserPoint() : theta(NAN) { markInvalid(); }

		/** This function returns true if the reading is valid.
		 */
		inline bool isValid() const { return reading != 0; }
		
		/// After calling this function, isValid will return true.
		void markInvalid()
		{
			reading=0; 
			intensity=NAN;
			trueAlpha=NAN;
		}
		
		bool hasTrueAlpha() const { return trueAlpha != NAN; }
		
		/// Returns a PolarPoint.
		RDK2::Geometry::PolarPoint ppoint() const { return RDK2::Geometry::PolarPoint(theta, reading); }
	};
	
	typedef std::vector<LaserPoint> LaserPoints;
	LaserPoints points;
	
	/// Constructor
	LaserData() : BaseSensorData("ROBOTLASER1"),
		// Fill with "safe" values
		minReading(0), 
		maxReading(0), // FIXME: AC: non dovrebbe essere a +INF
		minTheta(-M_PI/2), 
		maxTheta(+M_PI/2),
		roll(0),
		pitch(0),
		laserPan(0)
	{
	}
	
	
	/// @deprecated
	/** @brief Converts all internal measures from mm to meters
	 */
	void mm2m();
	
	/** @brief Sets the Roll (angle around z axis), Pitch (angle around y axis), Yaw (angle around x axis)
	 * angles of the laser range scanner sensor in the case is mounted on a Tilt Joint
	 */
	inline void setLaserRollPitchYaw(double roll=0, double pitch=0, double yaw=0){laserPose.theta=roll; laserPose.phi=pitch; laserPose.gamma=yaw;}
	
	/** @brief Returns the Roll angle of the laser range scanner sensor
	 * Rotation around the vertical axis (z) is called roll. 
	*/
	inline double getLaserRoll(){return laserPose.theta;}
	
	/** @brief Returns the Pitch angle of the laser range scanner sensor.
	 * Rotation around the side-to-side (y) axis is called pitch. 
	 */
	inline double getLaserPitch(){return laserPose.phi;}
	
	/** @brief Returns the Yaw angle of the laser range scanner sensor
	 * Rotation around the bottom to front axis (x) is called yaw. 
	 */
	inline double getLaserYaw(){return laserPose.gamma;}
	
	
	/** @brief Returns the index of the ray that is nearest to \c theta.
	 *  Returns the index of the ray that is nearest to \c theta, 
	 *  that is, \f$ round(\frac{\theta - \theta_{min}}{d}) \f$
	 *  in which \f$ d = \frac{\theta_{max} - \theta_{min}}{n_{points} - 1} \f$.
	 *  @param theta the angle for which the index is computed.
	 *  @return the index of the \c LaserPoint that is nearest to \c theta.
	 *  @warning the index can be outside the limits of the \c points vector.
	 */ 
	inline int getNearestIndex(double theta) const
	{ return (int) round((theta - minTheta) / getRayWidth()); }
	
	/** @brief Returns the distance between two rays (supposed to be the same for each ray).
	 *  Returns the distance between two rays (supposed to be the same for each ray),
	 *  that is, \f$ \frac{\theta_{max} - \theta_{min}}{n_{points} - 1} \f$.
	 *  @return the distance between two rays.
	 */
	inline double getRayWidth() const
	{ return (maxTheta - minTheta) / (points.size() - 1); }
	
	/** @brief Update the \c theta value of all laser points
	 * Update the \c theta value of all laser points, according to \f$ \theta_{min} \f$,
	 * \f$ \theta_{max} \f$ and the number of points.
	 */
	void updateThetas();
	
	/// Returns minimum range (values of 0 are ignored)
	double minRange() const;
	
	//Returns angle associated to minimum range
	int getMinAngle();
	
	/// True if same rays 
	bool raysEqualTo(const LaserData&ld);

	/// Number of points with reading != 0
	int numValidPoints() const;

	/// Returns true if the point is in the clear area swept by the laser 
	bool isInsideClearArea(RDK2::Geometry::Point2d world) const;

	/// These function need to know the orientation in space of the scan: 
	/// for this it uses the "estimate" field;
	
	/// You need to multiply these matrices by \f$ \frac{1}{\sigma^2} \f$ to
	/// get the actual Fisher's matrix.

	/// Returns the (unscaled) 2x2 Fisher's information matrix for (x,y).
	RDK2::Geometry::DMatrixD getFisherForXY();
	
	/// Returns the (unscaled) 1x1 Fisher's information matrix for theta.
	double getFisherForTheta();
	
	/// Returns the (unscaled) 3x3 Fisher's information matrix for (x,y,theta).
	RDK2::Geometry::DMatrixD getFisher();

	/// Returns the information matrix properly scaled by sigma.
	RDK2::Geometry::DMatrixD getFisher(double sigma) {
		return getFisher() * (1/(sigma*sigma));	
	}

	/// Trhows if Fisher's matrix is singular (under constrained situation) 
	RDK2::Geometry::DMatrixD getCRB(double sigma) 
		throw(RDK2::Geometry::DNotInvertibleMatrixException)
	{
		return getFisher(sigma).inv(); 
	}

	
	///
	/// Housekeeping
	///
	
	/// Explicit copy operation: in the past there have been
	/// some Strange Things going on with the default copy-constructors,
	/// even though this class is very simple.
	void copy(const LaserData&);
	static LaserParser laserParser;
	Parser* getParser() const ;
};

/** Extracts oriented points from a carmen scan using regression. Note that
	it does not care if some rays are set to 0. */
void laserdata2avv(
	const LaserData& scan, int nregpoints, double maxDist, RDK2::Geometry::AVV&avv);
 

}} // end namespace

#endif

