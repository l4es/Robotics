/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @copyright  2010 Manfred Kroehnert
*/

#include "RobotNodeFixedFactory.h"
#include "RobotNode.h"
#include "RobotNodeFixed.h"


namespace VirtualRobot
{

    RobotNodeFixedFactory::RobotNodeFixedFactory()
    {
    }


    RobotNodeFixedFactory::~RobotNodeFixedFactory()
    {
    }

    /**
     * This method creates a VirtualRobot::RobotNodeFixed.
     *
     * \return instance of VirtualRobot::RobotNodeFixed.
     */
    RobotNodePtr RobotNodeFixedFactory::createRobotNode(RobotPtr robot, const std::string& nodeName, VisualizationNodePtr visualizationModel, CollisionModelPtr collisionModel, float limitLow, float limitHigh, float jointValueOffset, const Eigen::Matrix4f& preJointTransform, const Eigen::Vector3f& axis, const Eigen::Vector3f& translationDirection, const SceneObject::Physics& p, RobotNode::RobotNodeType rntype) const
    {
        RobotNodePtr robotNode(new RobotNodeFixed(robot, nodeName, preJointTransform, visualizationModel, collisionModel, p, CollisionCheckerPtr(), rntype));

        return robotNode;
    }


    /**
     * This method creates a VirtualRobot::RobotNodeFixed from DH parameters.
     *
     * \return instance of VirtualRobot::RobotNodeFixed.
     */
    RobotNodePtr RobotNodeFixedFactory::createRobotNodeDH(RobotPtr robot, const std::string& nodeName, VisualizationNodePtr visualizationModel, CollisionModelPtr collisionModel, float limitLow, float limitHigh, float jointValueOffset, const DHParameter& dhParameters, const SceneObject::Physics& p, RobotNode::RobotNodeType rntype) const
    {
        RobotNodePtr robotNode(new RobotNodeFixed(robot, nodeName, dhParameters.aMM(), dhParameters.dMM(), dhParameters.alphaRadian(), dhParameters.thetaRadian(), visualizationModel, collisionModel, p, CollisionCheckerPtr(), rntype));

        return robotNode;
    }

    /**
     * register this class in the super class factory
     */
    RobotNodeFactory::SubClassRegistry RobotNodeFixedFactory::registry(RobotNodeFixedFactory::getName(), &RobotNodeFixedFactory::createInstance);


    /**
     * \return "fixed"
     */
    std::string RobotNodeFixedFactory::getName()
    {
        return "fixed";
    }


    /**
     * \return new instance of RobotNodeFixedFactory.
     */
    boost::shared_ptr<RobotNodeFactory> RobotNodeFixedFactory::createInstance(void*)
    {
        boost::shared_ptr<RobotNodeFixedFactory> fixedNodeFactory(new RobotNodeFixedFactory());
        return fixedNodeFactory;
    }

} // namespace VirtualRobot
