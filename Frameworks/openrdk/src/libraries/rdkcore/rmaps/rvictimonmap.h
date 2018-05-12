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

#ifndef RDK2_RMAPS_RVICTIMONMAP
#define RDK2_RMAPS_RVICTIMONMAP

#include <rdkcore/object/object.h>
#include <rdkcore/geometry/point.h>
#include <rdkcore/container/container.h>
#include <rdkcore/rgraphics/color.h>
#include <rdkcore/rgraphics/rimage.h>
#include <rdkcore/rgeometry/rpoint2od.h>
#include <rdkcore/rgeometry/rpoint2dvector.h>
#include <rdkcore/rgraphics/color.h>
#include <rdkcore/time/time.h>

#include "ritemonmapvector.h"

namespace RDK2 { namespace RMaps {
	
using RDK2::RGraphics::RgbColor;
using namespace RDK2::Geometry;
using namespace RDK2::RGeometry;
using namespace RDK2::RGraphics;
using namespace RDK2::RPrimitive;
using namespace RDK2::Time;

struct RVictimOnMap : public RItemOnMap {
	RVictimOnMap();
	~RVictimOnMap();

	Point2od pose;
	double zPosition;

	double theta, dist;		// bearing and distance with respect to the robot pose

	Point2od robotPose;
	RPoint2dVector pathToReach;
	
	bool confirmedOp, confirmedAI;			// operator or AI confirmed (or is sure it is a false positive)
	bool falsePositiveOp, falsePositiveAI;	// the victim is a false positive 
											// (the victim can either be confirmed XOR a false positive)
	double reliabilityOp, reliabilityAI;	// how much operator or AI is sure that it is a victim
	string idOp, idAI;						// numeric tag or id of the victim ("John", "Bob", 10, 20, A, C, ecc.)
	string descriptionOp, descriptionAI;	// description (color, shape, etc.)
	string motionOp, motionAI;				// either True/False or Moving Appendages, etc.
	double temperatureOp, temperatureAI;	// use Celsius degrees
	string soundOp, soundAI;				// either True/False or kind of sound (voice, beacons, tapping)
	string situationOp, situationAI;		// situation is one among "Surface", "Trapped", "Void", "Entombed"
	string statusOp, statusAI;				// conscious or not
	string partsOp, partsAI;				// space separated list of parts seen 
											// (in USARSim can be "Head", "Arm", "Hand", "Chest", "Pelvis", "Leg", "Foot")
	vector<int> seenPartsCache;				// parts currently seen (to compute mean)
	double computeSeenPartsMean();
	
	Timestamp timestamp;					// timestamp when the victim has been confirmed (robotPose is relative to this ts)
						
	string getLogLine();					// line in carmen format of the victim found
	
	RImage* snapshotOp;						// snapshot of the victim taken by operator
	RImage* snapshotAI;						// snapshot of the victim taken by AI

	vector<double> temperatureCache;		// used to compute temperature mean and variance
	vector<Point2od> poseCache;				// used to compute position mean and variance
	
	double computeTemperatureMean();
	Point2od computePoseMean();
	
	void read(Reader* r) throw (ReadingException);
	void write(Writer* w) const throw (WritingException);
	RDK2::Object* clone() const;	  
};

}} // namespace

#endif
