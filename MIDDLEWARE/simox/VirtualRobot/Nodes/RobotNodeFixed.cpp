
#include "RobotNodeFixed.h"
#include "../Robot.h"
#include "../Visualization//VisualizationNode.h"
#include "../CollisionDetection/CollisionModel.h"
#include <cmath>
#include <algorithm>

#include "../VirtualRobotException.h"

namespace VirtualRobot
{

    RobotNodeFixed::RobotNodeFixed(RobotWeakPtr rob,
                                   const std::string& name,
                                   const Eigen::Matrix4f& preJointTransform,
                                   VisualizationNodePtr visualization,
                                   CollisionModelPtr collisionModel,
                                   const SceneObject::Physics& p,
                                   CollisionCheckerPtr colChecker,
                                   RobotNodeType type
                                  ) : RobotNode(rob, name, 0.0f, 0.0f, visualization, collisionModel, 0.0f, p, colChecker, type)
    {
        optionalDHParameter.isSet = false;
        this->localTransformation = preJointTransform;
        checkValidRobotNodeType();
    }

    RobotNodeFixed::RobotNodeFixed(RobotWeakPtr rob,
                                   const std::string& name,
                                   float a, float d, float alpha, float theta,
                                   VisualizationNodePtr visualization,
                                   CollisionModelPtr collisionModel,
                                   const SceneObject::Physics& p,
                                   CollisionCheckerPtr colChecker,
                                   RobotNodeType type
                                  ) : RobotNode(rob, name, 0.0f, 1.0f, visualization, collisionModel, 0.0f, p, colChecker, type)
    {
        initialized = false;
        optionalDHParameter.isSet = true;
        optionalDHParameter.setAInMM(a);
        optionalDHParameter.setDInMM(d);
        optionalDHParameter.setAlphaRadian(alpha, true);
        optionalDHParameter.setThetaRadian(theta, true);

        // compute DH transformation matrices
        Eigen::Matrix4f RotTheta = Eigen::Matrix4f::Identity();
        RotTheta(0, 0) = cos(theta);
        RotTheta(0, 1) = -sin(theta);
        RotTheta(1, 0) = sin(theta);
        RotTheta(1, 1) = cos(theta);
        Eigen::Matrix4f TransD = Eigen::Matrix4f::Identity();
        TransD(2, 3) = d;
        Eigen::Matrix4f TransA = Eigen::Matrix4f::Identity();
        TransA(0, 3) = a;
        Eigen::Matrix4f RotAlpha = Eigen::Matrix4f::Identity();
        RotAlpha(1, 1) = cos(alpha);
        RotAlpha(1, 2) = -sin(alpha);
        RotAlpha(2, 1) = sin(alpha);
        RotAlpha(2, 2) = cos(alpha);

        this->localTransformation = RotTheta * TransD * TransA * RotAlpha;
    }

    RobotNodeFixed::~RobotNodeFixed()
    {
    }

    bool RobotNodeFixed::initialize(SceneObjectPtr parent, const std::vector<SceneObjectPtr>& children)
    {
        return RobotNode::initialize(parent, children);
    }

    void RobotNodeFixed::updateTransformationMatrices(const Eigen::Matrix4f& parentPose)
    {
        this->globalPose = parentPose * getLocalTransformation();
    }

    void RobotNodeFixed::print(bool printChildren, bool printDecoration) const
    {
        if (printDecoration)
        {
            cout << "******** RobotNodeFixed ********" << endl;
        }

        RobotNode::print(false, false);

        if (printDecoration)
        {
            cout << "******** End RobotNodeFixed ********" << endl;
        }


        std::vector< SceneObjectPtr > children = this->getChildren();

        if (printChildren)
        {
            std::for_each(children.begin(), children.end(), boost::bind(&SceneObject::print, _1, true, true));
        }
    }


    RobotNodePtr RobotNodeFixed::_clone(const RobotPtr newRobot, const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel, CollisionCheckerPtr colChecker, float scaling)
    {
        ReadLockPtr lock = getRobot()->getReadLock();

        RobotNodePtr result;

        Physics p = physics.scale(scaling);

        if (optionalDHParameter.isSet)
        {
            result.reset(new RobotNodeFixed(newRobot, name, optionalDHParameter.aMM()*scaling, optionalDHParameter.dMM()*scaling, optionalDHParameter.alphaRadian(), optionalDHParameter.thetaRadian(), visualizationModel, collisionModel, p, colChecker, nodeType));
        }
        else
        {
            Eigen::Matrix4f lt = getLocalTransformation();
            lt.block(0, 3, 3, 1) *= scaling;
            result.reset(new RobotNodeFixed(newRobot, name, lt, visualizationModel, collisionModel, p, colChecker, nodeType));
        }

        return result;
    }

    void RobotNodeFixed::checkValidRobotNodeType()
    {
        RobotNode::checkValidRobotNodeType();
        THROW_VR_EXCEPTION_IF(nodeType == Joint, "RobotNodeFixed not compatible with JointNode");
    }

    std::string RobotNodeFixed::_toXML(const std::string& modelPath)
    {
        return std::string();
    };


} // namespace
