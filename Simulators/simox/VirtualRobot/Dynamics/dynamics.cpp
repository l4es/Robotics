#include <rbdl/rbdl.h>
#include "dynamics.h"
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Nodes/RobotNodeFactory.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Units.h>

#include <Eigen/Dense>

#include <math.h>

#include <string>
#include <iostream>

#include <rbdl/rbdl_utils.h>


using std::cout;
using std::cin;
using namespace VirtualRobot;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;






VirtualRobot::Dynamics::Dynamics(RobotNodeSetPtr rns) : rns(rns) {
   if(!rns)
   {
       THROW_VR_EXCEPTION("RobotNodeSetPtr is zero pointer");
   }

    gravity = Vector3d(0, 0,-9.81);
    model = boost::shared_ptr<RigidBodyDynamics::Model>(new Model());

    model->gravity = gravity;

    if (!rns->isKinematicChain())
        THROW_VR_EXCEPTION("RobotNodeSet is not a kinematic chain!")

    RobotNodePtr root = rns->getKinematicRoot();
    //cout << "Root name: "<<root->getName()<<endl;

    //Dynamics::toRBDLRecursive(model, root, root->getGlobalPose(), Eigen::Matrix4f::Identity());
    Dynamics::toRBDL(model, root);
}

Eigen::VectorXd Dynamics::getInverseDynamics(Eigen::VectorXd q, Eigen::VectorXd qdot, Eigen::VectorXd qddot)
{
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(Dynamics::model->dof_count);
    InverseDynamics(*model.get(), q, qdot, qddot, tau);
    return tau;
}

Eigen::VectorXd Dynamics::getForwardDynamics(Eigen::VectorXd q, Eigen::VectorXd qdot, Eigen::VectorXd tau)
{
    Eigen::VectorXd qddot = Eigen::VectorXd::Zero(Dynamics::model->dof_count);
    ForwardDynamics(*model.get(), q, qdot, tau, qddot);
    return qddot;
}


Eigen::VectorXd Dynamics::getGravityMatrix(Eigen::VectorXd q, int nDof)
{

    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(nDof);
    Eigen::VectorXd qddot = Eigen::VectorXd::Zero(nDof);

    Eigen::VectorXd tauGravity = Eigen::VectorXd::Zero(Dynamics::model->dof_count);
    InverseDynamics(*model.get(), q, qdot, qddot, tauGravity);
    return tauGravity;
}

Eigen::VectorXd Dynamics::getCoriolisMatrix(Eigen::VectorXd q, Eigen::VectorXd qdot, int nDof)
{

    Eigen::VectorXd qddot = Eigen::VectorXd::Zero(nDof);
    Eigen::VectorXd tauGravity = getGravityMatrix(q, nDof);
    Eigen::VectorXd tau= Eigen::VectorXd::Zero(Dynamics::model->dof_count);

    Eigen::MatrixXd tauCoriolis = Eigen::VectorXd::Zero(Dynamics::model->dof_count);
    InverseDynamics(*model.get(), q, qdot, qddot, tau);
    tauCoriolis=tau-tauGravity;
    return tauCoriolis;
}



Eigen::MatrixXd Dynamics::getInertiaMatrix(Eigen::VectorXd q)
{
    Eigen::MatrixXd inertia = Eigen::MatrixXd::Zero(model->dof_count, model->dof_count);
    CompositeRigidBodyAlgorithm(*model.get(), q, inertia);
    return inertia;
}

void Dynamics::setGravity(Eigen::Vector3d gravity)
{
    model->gravity = gravity;
}

int Dynamics::getnDoF()
{
    return model->dof_count;
}

void Dynamics::print()
{
    std::string result = RigidBodyDynamics::Utils::GetModelHierarchy (*model.get());
    cout << "RBDL hierarchy of RNS:" << rns->getName() << endl;
    cout << result;
    result = RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview (*model.get());
    cout << "RBDL origins of RNS:" << rns->getName() << endl;
    cout << result;
    result = RigidBodyDynamics::Utils::GetModelDOFOverview (*model.get());
    cout << "RBDL DoF of RNS:" << rns->getName() << endl;
    cout << result;
}

// this method just selects the first node with an attached mass that is no Joint
RobotNodePtr Dynamics::checkForConnectedMass(RobotNodePtr node)
{
    if (!node)
        return RobotNodePtr();

    BOOST_FOREACH(SceneObjectPtr child, node->getChildren())
    {
        RobotNodePtr childPtr = boost::dynamic_pointer_cast<RobotNode>(child);
        if(     childPtr != 0 &&                // existing
                childPtr->getMass()>0 &&        // has mass
                (childPtr->isTranslationalJoint() // is translational joint
                 || (!childPtr->isTranslationalJoint() && !childPtr->isRotationalJoint()))) // or fixed joint
        {
            return childPtr;
        }
    }
    BOOST_FOREACH(SceneObjectPtr child, node->getChildren())
    {
        RobotNodePtr rnPtr = boost::dynamic_pointer_cast<RobotNode>(child);
        RobotNodePtr childPtr;
        if (rnPtr && !rnPtr->isRotationalJoint()) // break recursion if child is rot joint
            childPtr = checkForConnectedMass(rnPtr);
        if (childPtr)
            return childPtr;
    }
    return RobotNodePtr();
}

// rbdl: (trafo->joint->body) -> (trafo->joint->body) -> (trafo->joint->body) ...
void Dynamics::toRBDLRecursive(boost::shared_ptr<RigidBodyDynamics::Model> model, RobotNodePtr node, Eigen::Matrix4f accumulatedTransformPreJoint, Eigen::Matrix4f accumulatedTransformPostJoint, RobotNodePtr jointNode, int parentID)
{
    VR_ASSERT(model);
    VR_ASSERT(node);
    int nodeID = parentID; // might be overwritten when adding new body
    float mass = node->getMass();



    if (!jointNode)
    {
        accumulatedTransformPreJoint *= node->getLocalTransformation();
        if (Dynamics::rns->hasRobotNode(node) && (node->isRotationalJoint() || node->isTranslationalJoint()))
            jointNode = node;
    }
    else
    {
        accumulatedTransformPostJoint *= node->getLocalTransformation();
        if (Dynamics::rns->hasRobotNode(node) && (node->isRotationalJoint() || node->isTranslationalJoint()))
        {
            VR_ERROR << "Skipping joint " << node->getName() << ": multiple joints in row without masses inbetween..." << endl;
        }
    }


    if (mass>0 && Dynamics::rns->hasRobotNode(node))
    {
        // create a body
        //Vector3d com = node->getCoMLocal().cast<double>() / 1000; // divide by 1000 because Simox defines lengths in mm while the RBDL defines lengths in m
        //Matrix3d inertia = node->getInertiaMatrix().cast<double>();

        // apply postJoint transform
        Eigen::Vector4f comTr;
        comTr.head(3) = node->getCoMLocal();
        comTr(3) = 1.0f;
        Vector3d com = (accumulatedTransformPostJoint * comTr).head(3).cast<double>() / 1000.0;

        // convert inertia from node to jointFrame (I_new = R * I_old * R^T)
        Eigen::Matrix3f trafo = accumulatedTransformPostJoint.block(0,0,3,3);
        Eigen::Matrix3f inertia2 = trafo * node->getInertiaMatrix() * trafo.transpose();
        Matrix3d inertia = inertia2.cast<double>();


        Body body = Body(mass, com, inertia);

        Matrix3d spatial_rotation = accumulatedTransformPreJoint.block(0, 0, 3, 3).cast<double>();
        Vector3d spatial_translation = accumulatedTransformPreJoint.col(3).head(3).cast<double>() / 1000.0;
        SpatialTransform spatial_transform = SpatialTransform(spatial_rotation, spatial_translation);
        cout << "****** spatial_translation: " << spatial_translation.transpose() << endl;

        // joint
        Joint joint = Joint(JointTypeFixed);

        if (jointNode && jointNode->isRotationalJoint())
        {
            JointType joint_type = JointTypeRevolute;
            boost::shared_ptr<RobotNodeRevolute> rev = boost::dynamic_pointer_cast<RobotNodeRevolute>(jointNode);
            VR_ASSERT(rev);
            Vector3d joint_axis = rev->getJointRotationAxisInJointCoordSystem().cast<double>();

            joint = Joint(joint_type, joint_axis);

            VR_INFO << "Adding Rotational Joint" << endl;
        }
        else if (jointNode && jointNode->isTranslationalJoint())
        {
            JointType joint_type = JointTypePrismatic;
            boost::shared_ptr<RobotNodePrismatic> prism = boost::dynamic_pointer_cast<RobotNodePrismatic>(jointNode);
            Vector3d joint_axis = prism->getJointTranslationDirectionJointCoordSystem().cast<double>();

            joint = Joint(joint_type, joint_axis);

             VR_INFO << "Adding Translational Joint" << endl;
        }
        std::string nodeName;
        if (jointNode)
            nodeName = jointNode->getName();
        else
            nodeName = node->getName();

        nodeID = model->AddBody(parentID, spatial_transform, joint, body, nodeName);
        this->identifierMap[nodeName] = nodeID;

        cout << "New body:" << node->getName() << ", " << nodeID << " :" <<endl; // Debugging Info
        cout << "** SPATIAL TRAFO: " << endl << spatial_transform.toMatrix() << endl;
        cout << "** MASS: " << body.mMass << endl;
        cout << "** COM: " << body.mCenterOfMass.transpose() << endl;
        cout << "** INERTIA: " << endl << body.mInertia << endl;
        cout << "** mIsVirtual: " << body.mIsVirtual << endl;
        if (jointNode)
            cout << "** Joint: " << jointNode->getName() << endl;
        else
            cout << "** Joint: none" << endl;
        if (joint.mJointAxes)
            cout << "** JOINT AXES " << endl << *(joint.mJointAxes) << endl;

        // reset pre and post trafos and jointNode
        accumulatedTransformPreJoint = Eigen::Matrix4f::Identity();
        accumulatedTransformPostJoint = Eigen::Matrix4f::Identity();
        jointNode.reset();

    } /*else
    {

        // node without mass, check how to proceed

    }*/

    std::vector<SceneObjectPtr> children = node->getChildren();

    BOOST_FOREACH(SceneObjectPtr child, children)
    {
        boost::shared_ptr<RobotNode> childRobotNode = boost::dynamic_pointer_cast<RobotNode>(child);

        if (childRobotNode) // if cast returns 0 pointer, child is a sensor and can be omitted. also, child must be contained in the robotnodeset
        {
            //if (Dynamics::rns->hasRobotNode(childRobotNode))
            //{
                toRBDLRecursive(model, childRobotNode, accumulatedTransformPreJoint, accumulatedTransformPostJoint, jointNode, nodeID);
            //} else
            //{
            //    VR_INFO << "skipping RN " << childRobotNode->getName() << " since it is not part of RNS" << endl;
            //}
        }
    }

}



void Dynamics::toRBDL(boost::shared_ptr<RigidBodyDynamics::Model> model, RobotNodePtr node, RobotNodePtr parentNode, int parentID)
{
    RobotNodePtr physicsFromChild;
    int nodeID = parentID;
    // need to define body, joint and spatial transform
    // body first
    float mass = node->getMass();
    Vector3d com = node->getCoMLocal().cast<double>() / 1000; // divide by 1000 because Simox defines lengths in mm while the RBDL defines lengths in m
    Matrix3d inertia = node->getInertiaMatrix().cast<double>();


    // if mass is 0, check children for translational joint. In case there is one, take its mass, com and inertia. Probably a bit hacky right now!
    if (fabs(mass) < 0.000001)
    {
        // todo: throw an error, when multiple connected masses are present, currently just the first is chosen...
        RobotNodePtr childPtr = checkForConnectedMass(node);
        if (childPtr)
        {
            cout << "Joint without mass (" << node->getName() << "), using mass of " << childPtr->getName() << endl;
            //Vector3d fromNode = childPtr->getTransformationFrom(node).col(3).head(3).cast<double>() / 1000;
            mass = childPtr->getMass();
            com = node->toLocalCoordinateSystemVec(childPtr->getCoMGlobal()).cast<double>() / 1000;//childPtr->getCoMLocal().cast<double>() / 1000 + fromNode; // take pre-joint transformation of translational joint into consideration!

            // convert inertia from child to current frame (I_new = R * I_old * R^T)
            Eigen::Matrix3f trafo = node->toLocalCoordinateSystem(childPtr->getGlobalPose()).block(0,0,3,3);
            Eigen::Matrix3f inertia2 = trafo * childPtr->getInertiaMatrix() * trafo.transpose();
            inertia = inertia2.cast<double>();

            physicsFromChild = childPtr;
        }
    }

    Body body = Body(mass, com, inertia);


    // spatial transform next
    Eigen::Matrix4d trafo = Eigen::Matrix4d::Identity();

    if (parentNode)
    {
        trafo = node->getTransformationFrom(parentNode).cast<double>();
        //cout <<"parent Node: "<< parentNode->getName()<<endl;
        //cout << "transformation RBDL" << endl << trafo << endl;
    }
    else if (!parentNode)
    {
        trafo = node->getGlobalPose().cast<double>();
        /*
        if (node->getParent())
            trafo = node->getParent()->getGlobalPose().cast<double>();
        else if (node->getRobot())
            trafo = node->getRobot()->getGlobalPose().cast<double>();*/
    }

    Matrix3d spatial_rotation = trafo.block(0, 0, 3, 3);
    Vector3d spatial_translation = trafo.col(3).head(3) / 1000;

    cout << "****** spatial_translation: " << spatial_translation.transpose() << endl;

    SpatialTransform spatial_transform = SpatialTransform(spatial_rotation, spatial_translation);

    // last, joint
    Joint joint = Joint(JointTypeFixed);

    if (node->isRotationalJoint())
    {
        JointType joint_type = JointTypeRevolute;
        boost::shared_ptr<RobotNodeRevolute> rev = boost::dynamic_pointer_cast<RobotNodeRevolute>(node);
        Vector3d joint_axis = rev->getJointRotationAxisInJointCoordSystem().cast<double>();

        joint = Joint(joint_type, joint_axis);

        cout << "Rotational Joint added:" << endl;
    }
    else if (node->isTranslationalJoint())
    {
        JointType joint_type = JointTypePrismatic;
        boost::shared_ptr<RobotNodePrismatic> prism = boost::dynamic_pointer_cast<RobotNodePrismatic>(node);
        Vector3d joint_axis = prism->getJointTranslationDirectionJointCoordSystem().cast<double>();

        joint = Joint(joint_type, joint_axis);

         cout << "translational Joint added" << endl;
    }

    if (joint.mJointType != JointTypeFixed)
    {
        nodeID = model->AddBody(parentID, spatial_transform, joint, body, node->getName());
        this->identifierMap[node->getName()] = nodeID;

        cout << "New body:" << node->getName() << ", " << nodeID << " :" <<endl; // Debugging Info
        cout << "** SPATIAL TRAFO: " << endl << spatial_transform.toMatrix() << endl;
        cout << "** MASS: " << body.mMass << endl;
        cout << "** COM: " << body.mCenterOfMass.transpose() << endl;
        cout << "** INERTIA: " << endl << body.mInertia << endl;
        cout << "** mIsVirtual: " << body.mIsVirtual << endl;
        if (joint.mJointAxes)
            cout << "** JOINT AXES " << endl << *(joint.mJointAxes) << endl;
    }

    std::vector<SceneObjectPtr> children;


    // pick correct children to proceed the recursion
    if(physicsFromChild != 0)
    {
        children = physicsFromChild->getChildren();
    }
    else
    {
        children = node->getChildren();
    }

    BOOST_FOREACH(SceneObjectPtr child, children)
    {
        boost::shared_ptr<RobotNode> childRobotNode = boost::dynamic_pointer_cast<RobotNode>(child);

        if (childRobotNode != 0 && Dynamics::rns->hasRobotNode(childRobotNode)) // if cast returns 0 pointer, child is a sensor and can be omitted. also, child must be contained in the robotnodeset
        {
            if (joint.mJointType == JointTypeFixed) // if current node is fixed, make its parent the parent of the next recursion step and thereby skip it
            {
                node = parentNode;
            }
            toRBDL(model, childRobotNode, node, nodeID);

        }
    }

    return;
}
