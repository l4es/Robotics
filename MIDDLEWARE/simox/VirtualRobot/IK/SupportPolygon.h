/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2013 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_SupportPolygon_h_
#define _VirtualRobot_SupportPolygon_h_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Workspace/WorkspaceRepresentation.h>
#include <VirtualRobot/Workspace/Manipulability.h>
#include <VirtualRobot/IK/PoseQualityMeasurement.h>



namespace VirtualRobot
{

/*!
		The support polygon is defined through the contacts between a set of CollisionModels and the floor plane.
		In this implementation, contacts are defined as all surface points of the passed collision models which have 
        a distance to MathTools::FloorPlane that is lower than 5mm.
*/
class VIRTUAL_ROBOT_IMPORT_EXPORT SupportPolygon : public boost::enable_shared_from_this<SupportPolygon>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SupportPolygon(SceneObjectSetPtr contactModels);

	//! Recalculate contacts and compute polygon
	MathTools::ConvexHull2DPtr updateSupportPolygon(float maxFloorDist = 5.0f);

	MathTools::ConvexHull2DPtr getSupportPolygon2D();

	MathTools::Plane getFloorPlane();

	/*! 
		Computes quality index between 0 and 1 of the CoM projection of the robotNodeSet.
		1 means that the current 2D CoM lies at the center of the support polygon
		0 means it is outside.
	*/
    float getStabilityIndex(RobotNodeSetPtr rns, bool update=true);

    SceneObjectSetPtr getContactModels();

protected:

	float getSquaredDistLine(Eigen::Vector2f &p, Eigen::Vector2f &pt1, Eigen::Vector2f &pt2 );

    SceneObjectSetPtr contactModels;

	std::vector< CollisionModelPtr > colModels; 
	std::vector< Eigen::Vector2f > currentContactPoints2D; 

	MathTools::Plane floor;
	MathTools::ConvexHull2DPtr suportPolygonFloor;
};

typedef boost::shared_ptr<SupportPolygon> SupportPolygonPtr;

} // namespace VirtualRobot

#endif
