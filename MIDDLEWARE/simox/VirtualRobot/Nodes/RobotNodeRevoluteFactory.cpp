/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @copyright  2010 Manfred Kroehnert
*/

#include "RobotNodeRevoluteFactory.h"
#include "RobotNode.h"
#include "RobotNodeRevolute.h"


namespace VirtualRobot
{

    RobotNodeRevoluteFactory::RobotNodeRevoluteFactory()
    {
    }


    RobotNodeRevoluteFactory::~RobotNodeRevoluteFactory()
    {
    }


    /**
     * This method creates a VirtualRobot::RobotNodeRevolute.
     *
     * \return instance of VirtualRobot::RobotNodeRevolute.
     */
    RobotNodePtr RobotNodeRevoluteFactory::createRobotNode(RobotPtr robot, const std::string& nodeName, VisualizationNodePtr visualizationModel, CollisionModelPtr collisionModel, float limitLow, float limitHigh, float jointValueOffset, const Eigen::Matrix4f& preJointTransform, const Eigen::Vector3f& axis, const Eigen::Vector3f& translationDirection, const SceneObject::Physics& p, RobotNode::RobotNodeType rntype) const
    {
        RobotNodePtr robotNode(new RobotNodeRevolute(robot, nodeName, limitLow, limitHigh, preJointTransform, axis, visualizationModel, collisionModel, jointValueOffset, p,  CollisionCheckerPtr(), rntype));

        return robotNode;
    }


    /**
     * This method creates a VirtualRobot::RobotNodeRevolute from DH parameters.
     *
     * \return instance of VirtualRobot::RobotNodeRevolute.
     */
    RobotNodePtr RobotNodeRevoluteFactory::createRobotNodeDH(RobotPtr robot, const std::string& nodeName, VisualizationNodePtr visualizationModel, CollisionModelPtr collisionModel, float limitLow, float limitHigh, float jointValueOffset, const DHParameter& dhParameters, const SceneObject::Physics& p, RobotNode::RobotNodeType rntype) const
    {
        RobotNodePtr robotNode(new RobotNodeRevolute(robot, nodeName, limitLow, limitHigh, dhParameters.aMM(), dhParameters.dMM(), dhParameters.alphaRadian(), dhParameters.thetaRadian(), visualizationModel, collisionModel, jointValueOffset, p,  CollisionCheckerPtr(), rntype));

        return robotNode;
    }


    /**
     * register this class in the super class factory
     */
    RobotNodeFactory::SubClassRegistry RobotNodeRevoluteFactory::registry(RobotNodeRevoluteFactory::getName(), &RobotNodeRevoluteFactory::createInstance);


    /**
     * \return "revolute"
     */
    std::string RobotNodeRevoluteFactory::getName()
    {
        return "revolute";
    }


    /**
     * \return new instance of RobotNodeRevoluteFactory.
     */
    boost::shared_ptr<RobotNodeFactory> RobotNodeRevoluteFactory::createInstance(void*)
    {
        boost::shared_ptr<RobotNodeRevoluteFactory> revoluteNodeFactory(new RobotNodeRevoluteFactory());
        return revoluteNodeFactory;
    }

} // namespace VirtualRobot
