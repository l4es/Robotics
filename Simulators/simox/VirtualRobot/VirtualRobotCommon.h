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
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_Common_h_
#define _VirtualRobot_Common_h_

#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/CollisionDetection/CDManager.h>
#include <VirtualRobot/CollisionDetection/CollisionModelImplementation.h>
#include <VirtualRobot/CollisionDetection/CollisionCheckerImplementation.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/EndEffector/EndEffectorActor.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Nodes/RobotNodeFactory.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodePrismaticFactory.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/Nodes/RobotNodeRevoluteFactory.h>
#include <VirtualRobot/Nodes/RobotNodeFixed.h>
#include <VirtualRobot/Nodes/RobotNodeFixedFactory.h>
#include <VirtualRobot/Nodes/RobotNodeActuator.h>
#include <VirtualRobot/Nodes/ConditionedLock.h>
#include <VirtualRobot/Transformation/DHParameter.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/Visualization.h>
#include <VirtualRobot/Visualization/VisualizationNode.h>
#include <VirtualRobot/Visualization/ColorMap.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/XML/BaseIO.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/IK/IKSolver.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/IK/GenericIKSolver.h>
#include <VirtualRobot/IK/CoMIK.h>
#include <VirtualRobot/IK/PoseQualityMeasurement.h>
#include <VirtualRobot/IK/PoseQualityManipulability.h>
#include <VirtualRobot/Workspace/WorkspaceDataArray.h>
#include <VirtualRobot/Workspace/WorkspaceData.h>
#include <VirtualRobot/Workspace/WorkspaceRepresentation.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Workspace/VoxelTree6D.hpp>
#include <VirtualRobot/Workspace/VoxelTree6DElement.hpp>
#include <VirtualRobot/Workspace/WorkspaceGrid.h>
#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <VirtualRobot/Grasping/BasicGraspQualityMeasure.h>
#include <VirtualRobot/AbstractFactoryMethod.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotConfig.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Trajectory.h>
#include <VirtualRobot/KinematicChain.h>
#include <VirtualRobot/RobotFactory.h>
#include <VirtualRobot/SceneObject.h>
#include <VirtualRobot/SceneObjectSet.h>
#include <VirtualRobot/Scene.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/VirtualRobotImportExport.h>
//#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/BoundingBox.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Compression/CompressionRLE.h>
#include <VirtualRobot/Compression/CompressionBZip2.h>
#include <VirtualRobot/SphereApproximator.h>

#endif // _VirtualRobot_Common_h_
