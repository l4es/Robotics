
#include "RobotNodeRevolute.h"
#include "../Robot.h"
#include <cmath>
#include <algorithm>

#include <Eigen/Geometry>
#include "../VirtualRobotException.h"

namespace VirtualRobot
{


    RobotNodeRevolute::RobotNodeRevolute(RobotWeakPtr rob,
                                         const std::string& name,
                                         float jointLimitLo,
                                         float jointLimitHi,
                                         const Eigen::Matrix4f& preJointTransform,            //!< This transformation is applied before the translation of the joint is done
                                         const Eigen::Vector3f& axis,         //!< This is the direction of the translation
                                         VisualizationNodePtr visualization,
                                         CollisionModelPtr collisionModel,
                                         float jointValueOffset,
                                         const SceneObject::Physics& p,
                                         CollisionCheckerPtr colChecker,
                                         RobotNodeType type
                                        ) : RobotNode(rob, name, jointLimitLo, jointLimitHi, visualization, collisionModel, jointValueOffset, p, colChecker, type)
    {
        initialized = false;
        optionalDHParameter.isSet = false;
        this->localTransformation = preJointTransform;
        this->jointRotationAxis = axis;
        checkValidRobotNodeType();
    }

    RobotNodeRevolute::RobotNodeRevolute(RobotWeakPtr rob,
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

        this->localTransformation = RotTheta * TransD * TransA * RotAlpha;
        this->jointRotationAxis = Eigen::Vector3f(0, 0, 1);         // rotation around z axis
        checkValidRobotNodeType();
    }


    RobotNodeRevolute::~RobotNodeRevolute()
    {
    }

    bool RobotNodeRevolute::initialize(SceneObjectPtr parent, const std::vector<SceneObjectPtr>& children)
    {
        return RobotNode::initialize(parent, children);
    }

    void RobotNodeRevolute::updateTransformationMatrices(const Eigen::Matrix4f& parentPose)
    {
        Eigen::Affine3f tmpT(Eigen::AngleAxisf(jointValue + jointValueOffset, jointRotationAxis));
        globalPose = parentPose * getLocalTransformation() * tmpT.matrix();
    }

    void RobotNodeRevolute::print(bool printChildren, bool printDecoration) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();

        if (printDecoration)
        {
            cout << "******** RobotNodeRevolute ********" << endl;
        }

        RobotNode::print(false, false);

        cout << "* JointRotationAxis: " << jointRotationAxis[0] << ", " << jointRotationAxis[1] << ", " << jointRotationAxis[2] << endl;

        if (printDecoration)
        {
            cout << "******** End RobotNodeRevolute ********" << endl;
        }

        std::vector< SceneObjectPtr > children = this->getChildren();

        if (printChildren)
        {
            std::for_each(children.begin(), children.end(), boost::bind(&SceneObject::print, _1, true, true));
        }
    }

    RobotNodePtr RobotNodeRevolute::_clone(const RobotPtr newRobot, const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel, CollisionCheckerPtr colChecker, float scaling)
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        RobotNodePtr result;

        Physics p = physics.scale(scaling);

        if (optionalDHParameter.isSet)
        {
            result.reset(new RobotNodeRevolute(newRobot, name, jointLimitLo, jointLimitHi, optionalDHParameter.aMM()*scaling, optionalDHParameter.dMM()*scaling, optionalDHParameter.alphaRadian(), optionalDHParameter.thetaRadian(), visualizationModel, collisionModel, jointValueOffset, p, colChecker, nodeType));
        }
        else
        {
            Eigen::Matrix4f lt = getLocalTransformation();
            lt.block(0, 3, 3, 1) *= scaling;
            result.reset(new RobotNodeRevolute(newRobot, name, jointLimitLo, jointLimitHi, lt, jointRotationAxis, visualizationModel, collisionModel, jointValueOffset, p, colChecker, nodeType));
        }

        return result;
    }

    bool RobotNodeRevolute::isRotationalJoint() const
    {
        return true;
    }

    void RobotNodeRevolute::setJointRotationAxis(Eigen::Vector3f newAxis) {
        this->jointRotationAxis = newAxis;
    }

    Eigen::Vector3f RobotNodeRevolute::getJointRotationAxis(const SceneObjectPtr coordSystem) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        Eigen::Vector4f result4f = Eigen::Vector4f::Zero();
        result4f.segment(0, 3) = jointRotationAxis;
        result4f = globalPose * result4f;

        if (coordSystem)
        {
            //res = coordSystem->toLocalCoordinateSystem(res);
            result4f = coordSystem->getGlobalPose().inverse() * result4f;
        }

        return result4f.block(0, 0, 3, 1);
    }

    void RobotNodeRevolute::updateVisualizationPose(const Eigen::Matrix4f& globalPose, bool updateChildren /*= false*/)
    {
        RobotNode::updateVisualizationPose(globalPose, updateChildren);

        // compute the jointValue from pose
        // -> Not done any more, the RobotNdoeActuator sets the joint angle

        // jointRotationAxis is given in local joint coord system
        // -> we need the pose in joint coord system
        /*
        Eigen::Matrix4f initFrame;
        if (this->getParent())
            initFrame = this->getParent()->getGlobalPose() * getLocalTransformation();
        else
            initFrame = getLocalTransformation();

        Eigen::Matrix4f localPose = initFrame.inverse() * globalPose;

        Eigen::Vector3f localAxis;
        float angle;
        MathTools::eigen4f2axisangle(localPose,localAxis,angle);

        // check pos/neg direction
        if (jointRotationAxis.normalized().dot(localAxis.normalized()) > 0)
        {
            // pointing in opposite direction
            angle = -angle;
        }

        // consider offset
        jointValue = angle - jointValueOffset;*/
    }

    Eigen::Vector3f RobotNodeRevolute::getJointRotationAxisInJointCoordSystem() const
    {
        return jointRotationAxis;
    }

    void RobotNodeRevolute::checkValidRobotNodeType()
    {
        RobotNode::checkValidRobotNodeType();
        THROW_VR_EXCEPTION_IF(nodeType == Body || nodeType == Transform, "RobotNodeRevolute must be a JointNode or a GenericNode");
    }


   std::string RobotNodeRevolute::_toXML(const std::string& modelPath)
   {
        std::stringstream ss;
        ss << "\t\t<Joint type='revolute'>" << endl;
        ss << "\t\t\t<axis x='" << jointRotationAxis[0] << "' y='" << jointRotationAxis[1] << "' z='" << jointRotationAxis[2] << "'/>" << endl;
        ss << "\t\t\t<limits lo='" << jointLimitLo << "' hi='" << jointLimitHi << "' units='radian'/>" << endl;
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

float RobotNodeRevolute::getLMTC(float angle)
{

    return sqrt(2 - 2 * cos(angle));
    /*
    Eigen::Matrix4f fromParent = getTransformationFrom(getParent());
    Eigen::Matrix4f toChild = child->getTransformationFrom(child->getParent());

        while (propIt != propagatedJointValues.end())
        {
            ss << "\t\t\t<PropagateJointValue name='" << propIt->first << "' factor='" << propIt->second << "'/>" << endl;
            propIt++;
        }

        ss << "\t\t</Joint>" << endl;

        return ss.str();
    }

    return sqrt(b_squared);
    */
}

        //Eigen::Matrix4f fromParent = getTransformationFrom(getParent());
        //Eigen::Matrix4f toChild = child->getTransformationFrom(child->getParent());

float RobotNodeRevolute::getLMomentArm(float angle)
{
    float b = getLMTC(angle);

    return sqrt(4*pow(b,2)-pow(b,4))/(2*b);
    /*
    Eigen::Matrix4f fromParent = getTransformationFrom(getParent());
    Eigen::Matrix4f toChild = child->getTransformationFrom(child->getParent());

        THROW_VR_EXCEPTION_IF(a < 0.0000001 || c < 0.0000001, "Node has no spatial offset to either parent or child");

        float b_squared = a * a + c * c - 2 * a * c * cos(angle);

        return sqrt(b_squared);
    }


    return lMomentArm;
    */
}
        /*
        float a = fromParent.col(3).head(3).norm();
        float c = toChild.col(3).head(3).norm();

        THROW_VR_EXCEPTION_IF(a < 0.0000001 || c < 0.0000001, "Node has no spatial offset to either parent or child");

        float b = getLMTC(child, angle);

        float lMomentArm = sqrt(2 * (a * a * b * b + b * b * c * c + c * c * a * a) - (pow(a, 4) + pow(b, 4) + pow(c, 4))) / (2 * b);

        return lMomentArm;

    }*/

} // namespace
