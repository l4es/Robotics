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
#ifndef _VirtualRobot_FeetPosture_h_
#define _VirtualRobot_FeetPosture_h_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/RobotNodeSet.h>

namespace VirtualRobot
{

/*!
	A feet posture comprises information about 
		* the robot and the corresponding RobotNodeSets
		* The base node (eg the hip)
		* The TCPs of both feet
		* the Cartesian relation of both feet when applying the posture
*/
class VIRTUAL_ROBOT_IMPORT_EXPORT FeetPosture : public boost::enable_shared_from_this<FeetPosture>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	FeetPosture(RobotNodeSetPtr leftLeg, 
				RobotNodeSetPtr rightLeg, 
				Eigen::Matrix4f &transformationLeftToRightFoot, 
				RobotNodePtr baseNode,
				RobotNodePtr leftTCP = RobotNodePtr(),
				RobotNodePtr rightTCP = RobotNodePtr(),
				RobotNodeSetPtr rnsLeft2RightFoot = RobotNodeSetPtr()
				);

	virtual ~FeetPosture();

	RobotNodeSetPtr getLeftLeg();
	RobotNodeSetPtr getRightLeg();
	RobotNodeSetPtr getLeftLegCol();
	RobotNodeSetPtr getRightLegCol();
	RobotNodePtr getLeftTCP();
	RobotNodePtr getRightTCP();
	RobotNodePtr getBaseNode();
	RobotPtr getRobot();
	Eigen::Matrix4f getTransformationLeftToRightFoot();
	
	//! Initially the rns of left and right leg are used as collision models. Here you can set other sets for collision detection.
	void setCollisionCheck(RobotNodeSetPtr leftColModel, RobotNodeSetPtr rightColModel);

	//! The collision models are queried if there is a collision for the current config.
	bool icCurrentLegConfigCollisionFree();

	//! Optional: kinematic chain from left foot to waist to right foot (not supported by all kinematic structures)
	RobotNodeSetPtr getRNSLeft2RightFoot();
	void setRNSLeft2RightFoot(RobotNodeSetPtr rns);

	void print();
protected:

	RobotNodeSetPtr leftLeg;
	RobotNodeSetPtr rightLeg;
	RobotNodeSetPtr leftLegCol;
	RobotNodeSetPtr rightLegCol;
	RobotNodeSetPtr left2Right;
	Eigen::Matrix4f transformationLeftToRightFoot; 
	RobotNodePtr leftTCP;
	RobotNodePtr rightTCP;
	RobotNodePtr baseNode;

};

typedef boost::shared_ptr<FeetPosture> FeetPosturePtr;

} // namespace VirtualRobot

#endif
