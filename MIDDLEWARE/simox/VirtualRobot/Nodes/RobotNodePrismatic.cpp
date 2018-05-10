
#include "RobotNodePrismatic.h"
#include "../Robot.h"
#include <cmath>
#include <algorithm>

#include <Eigen/Geometry>
#include "../VirtualRobotException.h"

namespace VirtualRobot
{

    RobotNodePrismatic::RobotNodePrismatic(RobotWeakPtr rob,
                                           const std::string& name,
                                           float jointLimitLo,
                                           float jointLimitHi,
                                           const Eigen::Matrix4f& preJointTransform,
                                           const Eigen::Vector3f& translationDirection,
                                           VisualizationNodePtr visualization,
                                           CollisionModelPtr collisionModel,
                                           float jointValueOffset,
                                           const SceneObject::Physics& p,
                                           CollisionCheckerPtr colChecker,
                                           RobotNodeType type
                                          ) : RobotNode(rob, name, jointLimitLo, jointLimitHi, visualization, collisionModel, jointValueOffset, p, colChecker, type)
    {
        initialized = false;
        visuScaling = false;
        visuScaleFactor << 1.0f, 1.0f, 1.0f;
        optionalDHParameter.isSet = false;
        this->localTransformation = preJointTransform;
        this->jointTranslationDirection = translationDirection;
        checkValidRobotNodeType();
    }

    RobotNodePrismatic::RobotNodePrismatic(RobotWeakPtr rob,
                                           const std::string& name,
                                           float jointLimitLo,
                                           float jointLimitHi,
                                           float a, float d, float alpha, float theta,
                                           VisualizationNodePtr visualization,
                                           CollisionModelPtr collisionModel,
                                           float jointValueOffset,
                                           const SceneObject::Physics& p,
                                           CollisionCheckerPtr colChecker,
                                           RobotNodeType type
                                          ) : RobotNode(rob, name, jointLimitLo, jointLimitHi, visualization, collisionModel, jointValueOffset, p, colChecker, type)
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

        // fixed rotation around theta
        this->localTransformation = RotTheta * TransD * TransA * RotAlpha;
        // joint setup
        jointTranslationDirection = Eigen::Vector3f(0, 0, 1); // translation along the z axis
        checkValidRobotNodeType();
    }

    RobotNodePrismatic::~RobotNodePrismatic()
    {
    }


    bool RobotNodePrismatic::initialize(SceneObjectPtr parent, const std::vector<SceneObjectPtr>& children)
    {
        bool res = RobotNode::initialize(parent, children);
        unscaledLocalCoM = physics.localCoM;
        return res;
    }

    void RobotNodePrismatic::updateTransformationMatrices(const Eigen::Matrix4f& parentPose)
    {
        Eigen::Affine3f tmpT(Eigen::Translation3f((this->getJointValue() + jointValueOffset)*jointTranslationDirection));
        globalPose = parentPose * getLocalTransformation() * tmpT.matrix();

        if (visuScaling)
        {
            float scValue = 0;

            if (jointValueOffset == 0)
            {
                scValue = jointValue;
            }
            else
            {
                scValue = (jointValue) / jointValueOffset;
            }

            Eigen::Vector3f scVec;

            for (int i = 0; i < 3; i++)
            {
                scVec(i) = 1.0f + visuScaleFactor(i) * scValue;
            }

            if (visualizationModel)
            {
                visualizationModel->scale(scVec);
            }

            if (collisionModel)
            {
                collisionModel->scale(scVec);
            }

            for (int i = 0; i < 3; i++)
            {
                physics.localCoM(i) = unscaledLocalCoM(i) * scVec(i);
            }
        }
    }

    void RobotNodePrismatic::print(bool printChildren, bool printDecoration) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();

        if (printDecoration)
        {
            cout << "******** RobotNodePrismatic ********" << endl;
        }

        RobotNode::print(false, false);

        cout << "* jointTranslationDirection: " << jointTranslationDirection[0] << ", " << jointTranslationDirection[1] << "," << jointTranslationDirection[2] << endl;
        cout << "* VisuScaling: ";

        if (visuScaling)
        {
            cout << visuScaleFactor[0] << ", " << visuScaleFactor[1] << "," << visuScaleFactor[2] << endl;
        }
        else
        {
            cout << "disabled" << endl;
        }

        if (printDecoration)
        {
            cout << "******** End RobotNodePrismatic ********" << endl;
        }


        std::vector< SceneObjectPtr > children = this->getChildren();

        if (printChildren)
        {
            std::for_each(children.begin(), children.end(), boost::bind(&SceneObject::print, _1, true, true));
        }
    }

    RobotNodePtr RobotNodePrismatic::_clone(const RobotPtr newRobot, const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel, CollisionCheckerPtr colChecker, float scaling)
    {
        boost::shared_ptr<RobotNodePrismatic> result;
        ReadLockPtr lock = getRobot()->getReadLock();
        Physics p = physics.scale(scaling);

        if (optionalDHParameter.isSet)
        {
            result.reset(new RobotNodePrismatic(newRobot, name, jointLimitLo, jointLimitHi, optionalDHParameter.aMM()*scaling, optionalDHParameter.dMM()*scaling, optionalDHParameter.alphaRadian(), optionalDHParameter.thetaRadian(), visualizationModel, collisionModel, jointValueOffset, p, colChecker, nodeType));
        }
        else
        {
            Eigen::Matrix4f lt = getLocalTransformation();
            lt.block(0, 3, 3, 1) *= scaling;
            result.reset(new RobotNodePrismatic(newRobot, name, jointLimitLo, jointLimitHi, lt, jointTranslationDirection, visualizationModel, collisionModel, jointValueOffset, p, colChecker, nodeType));
        }

        if (result && visuScaling)
        {
            result->setVisuScaleFactor(visuScaleFactor);
        }

        return result;
    }

    bool RobotNodePrismatic::isTranslationalJoint() const
    {
        return true;
    }


    Eigen::Vector3f RobotNodePrismatic::getJointTranslationDirection(const SceneObjectPtr coordSystem) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        Eigen::Vector4f result4f = Eigen::Vector4f::Zero();
        result4f.segment(0, 3) = jointTranslationDirection;

        result4f = getGlobalPose() * result4f;

        if (coordSystem)
        {
            result4f = coordSystem->getGlobalPose().inverse() * result4f;
        }

        return result4f.segment(0, 3);
    }

    void RobotNodePrismatic::updateVisualizationPose(const Eigen::Matrix4f& globalPose, bool updateChildren /*= false*/)
    {
        RobotNode::updateVisualizationPose(globalPose, updateChildren);

        // compute the jointValue from pose
        // not done here any more: the RobotNdoeActuator is responsible for setting the jointValues
        /*
        Eigen::Matrix4f initFrame;
        if (this->getParent())
            initFrame = this->getParent()->getGlobalPose() * getLocalTransformation();
        else
            initFrame = getLocalTransformation();

        Eigen::Matrix4f localPose = initFrame.inverse() * globalPose;

        Eigen::Vector3f v = localPose.block(0,3,3,1);

        // project on directionVector
        MathTools::Line l(Eigen::Vector3f::Zero(),jointTranslationDirection);
        Eigen::Vector3f v_on_line = MathTools::nearestPointOnLine(l,v);
        float dist = v_on_line.norm();

        // check pos/neg direction
        if (v_on_line.normalized().dot(jointTranslationDirection.normalized()) < 0)
        {
            // pointing in opposite direction
            dist = -dist;
        }

        // consider offset
        jointValue = dist - jointValueOffset;
        */
    }

    Eigen::Vector3f RobotNodePrismatic::getJointTranslationDirectionJointCoordSystem() const
    {
        return jointTranslationDirection;
    }

    void RobotNodePrismatic::checkValidRobotNodeType()
    {
        RobotNode::checkValidRobotNodeType();
        THROW_VR_EXCEPTION_IF(nodeType == Body || nodeType == Transform, "RobotNodePrismatic must be a JointNode or a GenericNode");
    }

    std::string RobotNodePrismatic::_toXML(const std::string& modelPath)
    {
        std::stringstream ss;
        ss << "\t\t<Joint type='prismatic'>" << endl;
        ss << "\t\t\t<translationdirection x='" << jointTranslationDirection[0] << "' y='" << jointTranslationDirection[1] << "' z='" << jointTranslationDirection[2] << "'/>" << endl;
        ss << "\t\t\t<limits lo='" << jointLimitLo << "' hi='" << jointLimitHi << "' units='mm'/>" << endl;
        ss << "\t\t\t<MaxAcceleration value='" << maxAcceleration << "'/>" << endl;
        ss << "\t\t\t<MaxVelocity value='" << maxVelocity << "'/>" << endl;
        ss << "\t\t\t<MaxTorque value='" << maxTorque << "'/>" << endl;

        std::map< std::string, float >::iterator propIt = propagatedJointValues.begin();

        while (propIt != propagatedJointValues.end())
        {
            ss << "\t\t\t<PropagateJointValue name='" << propIt->first << "' factor='" << propIt->second << "'/>" << endl;
            propIt++;
        }

        ss << "\t\t</Joint>" << endl;
        return ss.str();
    }

    void RobotNodePrismatic::setVisuScaleFactor(Eigen::Vector3f& scaleFactor)
    {
        if (scaleFactor.norm() == 0.0f)
        {
            visuScaling = false;
        }
        else
        {
            visuScaling = true;
            visuScaleFactor = scaleFactor;
        }
    }



} // namespace
