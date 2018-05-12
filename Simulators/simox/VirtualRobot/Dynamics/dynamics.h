#ifndef _DYNAMICS_H_
#define _DYNAMICS_H__

#include "../VirtualRobotImportExport.h"

#include "../Nodes/RobotNode.h"
#include "../RobotNodeSet.h"
#include <rbdl/rbdl.h>


namespace VirtualRobot
{

    /** @brief
     * Encapsulates dynamics simulations based on the RBDL library for the virtual robot.
     *
     */
    class Dynamics
    {
    public:
        /// Creates a Dynamics object given a RobotNodeSet.
        /// The rns has to be completely connected (avoid missing RobotNodes).
        /// The rns should end with a RobotNode that has a mass>0 specified, otherwise the last joint can not be added to the internal RBDL model
        ///
        Dynamics(RobotNodeSetPtr rns);
        /// Calculates the Inverse Dynamics for given motion state defined by q, qdot and qddot
        Eigen::VectorXd getInverseDynamics(Eigen::VectorXd q, Eigen::VectorXd qdot, Eigen::VectorXd qddot);
        /// Calculates the joint space inertia matrix given a joint position vector q
        Eigen::VectorXd getGravityMatrix(Eigen::VectorXd q, int nDof);
        /// Calculates the joint space Gravity Matrix given a joint position vector q and Number of DOF
        Eigen::VectorXd getCoriolisMatrix(Eigen::VectorXd q, Eigen::VectorXd qdot, int nDof);
        /// Calculates the coriolis matrix given position vector q, velocity vector qdot and Number of DOF
        Eigen::VectorXd getForwardDynamics(Eigen::VectorXd q, Eigen::VectorXd qdot, Eigen::VectorXd tau);
        /// Calculates forward dynamics given position vector q velocity vector qdot and joint torques tau
        Eigen::MatrixXd getInertiaMatrix(Eigen::VectorXd q);
        /// Sets the gravity vector of the dynamics system
        void setGravity(Eigen::Vector3d gravity);
        /// returns the number of Degrees of Freedom of the dynamics system
        int getnDoF();

        int getIdentifier(std::string name){return identifierMap.at(name);}

        void print();

    protected:
        RobotNodeSetPtr rns;
        boost::shared_ptr<RigidBodyDynamics::Model> model;
        Eigen::Vector3d gravity;
        std::map<std::string,  int> identifierMap;

        RobotNodePtr checkForConnectedMass(RobotNodePtr node);
    private:
        void toRBDL(boost::shared_ptr<RigidBodyDynamics::Model> model, RobotNodePtr node, RobotNodePtr parentNode = RobotNodePtr(), int parentID = 0);
        void toRBDLRecursive(boost::shared_ptr<RigidBodyDynamics::Model> model, RobotNodePtr currentNode, Eigen::Matrix4f accumulatedTransformPreJoint, Eigen::Matrix4f accumulatedTransformPostJoint, RobotNodePtr jointNode = RobotNodePtr(), int parentID = 0);

    };

    typedef boost::shared_ptr<Dynamics> DynamicsPtr;
}

#endif
