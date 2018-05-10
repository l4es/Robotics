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
* @author     Stefan Ulbrich, Nikolaus Vahrenkamp
* @copyright  2011 Stefan Ulbrich, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#ifndef _VirtualRobot_DiffIK_h_
#define _VirtualRobot_DiffIK_h_

#include "../VirtualRobotImportExport.h"

#include "../Nodes/RobotNode.h"
#include "../RobotNodeSet.h"
#include "JacobiProvider.h"
#include "IKSolver.h"

#include <string>
#include <vector>



namespace VirtualRobot
{

    /*!
        @brief Encapsulates a differential inverse kinematics for the virtual robot.

        @details The aim of this class is to offer a convenient mechanism to solve the inverse kinematics of
        redundant robots. With this class it is possible to specify goals for multiple TCPs which will be
        be used simultaneously within the same optimization step. That way, the IK for two-armed manipulation
        can be solved where both arms share the same hip joint (see image).
        For each tcp, it is possible to select which components of the error vector should be optimized.
        One can restrict it to consider only the orientation, or only position, or even only the \f$z\f$-component, for instance.
        Any node of the virtual robot can be selected chosen tcp. For instance, the robot hand can be targed to reach for a position
            while its elbow should be lifted to a certain hight (see images and examples)

        Despite all higher-level functionality, the standard application can still be implemented very intuitively.
        Example for bimanual manipulation:
        @code
        // define a kinematic chain for bimanual manipulation.
        std::vector<RobotNodePtr> nBi;
        nBi.push_back(robot->getRobotNode(std::string("Shoulder 1 L")));
        nBi.push_back(robot->getRobotNode(std::string("Shoulder 1 R")));
        // ...
        RobotNodeSetPtr kcBi = RobotNodeSet::createRobotNodeSet(robot,std::string("jacobiTestBi"),nBi);

        // Create an IK algorithm instance. The world coordinate system is the frame of reference.
        DifferentialIK dIK (kcBi);

        // Set the target poses for the two end effectors.
        dIK.setGoal(targetPose,tcp,DifferentialIK::Position);
        dIK.setGoal(targetPose2,tcp2,DifferentialIK::Position);

        // compute the IK using 0.1 stepsize and a maximum of 50 steps.
        dIK.solveIK(0.1f);
        @endcode
        Example for lifted elbow:
        @code
        // should yield the kinematic chain of the left Arm
        RobotNodeSetPtr leftArm;
        // ...

        // Create an IK algorithm instance. The world coordinate system is the frame of reference.
        DifferentialIK dIK(leftArm);

        // should yield the target vector.
        Vector3f target_position;
        // ...

        // Set the target for the end effector.
        // This call sets the goal for the default tcp of the chain.
        // For vectors, the default is to optimize the position only.
        dIK.setGoal(target_position);

        // Now the vector should yield the height of the elbow;
        float elbow_target_z;
        target_position(2) = elbow_target_z;

        // Should be a reference to the elbow
        RobotNodePtr elbow;

        dIK.setGoal(target_position, elbow,DifferentialIK::Z);

        // compute the IK using 0.1 stepsize and a maximum of 50 steps.
        dIK.solveIK(0.1);

        // the joints of kcBi are already set to the result
        @endcode
    */

    class VIRTUAL_ROBOT_IMPORT_EXPORT DifferentialIK : public JacobiProvider, public boost::enable_shared_from_this<DifferentialIK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            @brief Initialize a Jacobian object.
            \param rns The robotNodes (i.e., joints) for which the Jacobians should be calculated.
            \param coordSystem The coordinate system in which the Jacobians are defined. By default the global coordinate system is used.
            \param invJacMethod The method for inverting the Jacobian
            \param invParam Only used when != 0.0f
        */
        DifferentialIK(RobotNodeSetPtr rns, RobotNodePtr coordSystem = RobotNodePtr(), JacobiProvider::InverseJacobiMethod invJacMethod = eSVD, float invParam = 0.0f);

        /*! @brief Sets the target position for (one of) the tcp(s).
            \param goal Target pose of the tcp.
            \param tcp The tcp joint that should be considered. By default the tcp joint that is defined in rns  in the constructor is used.
            \param mode Allows to include only a subset of the Cartesian coordinates in the Jacobian (e.g., X|Y if the z-component is unimportant)
            @param tolerancePosition The threshold when to accept a solution.
            @param toleranceRotation The threshold when to accept a solution in radians.
            @param performInitialization If multiple goals will be set, the internal initialization can be omitted in order to speed up the setup procedure (Ensure, to call initialize() after setting all goals).
        */
        virtual void setGoal(const Eigen::Matrix4f& goal, SceneObjectPtr tcp = SceneObjectPtr(), IKSolver::CartesianSelection mode = IKSolver::All, float tolerancePosition = 5.0f, float toleranceRotation = 3.0f / 180.0f * M_PI, bool performInitialization = true);


        /*! @brief Sets the target position for (one of) the tcp(s).
            \param goal Target position of the tcp.
            \param tcp The tcp joints that should be considered. By default the tcp joint that is defined in rns  in the constructor is used.
            \param mode Allows to include only a subset of the Cartesian coordinates in the Jacobian (e.g., X|Y if the z-component is unimportant)
            @param tolerancePosition The threshold when to accept a solution.
            @param toleranceRotation The threshold when to accept a solution in radians.
            @param performInitialization If multiple goals will be set, the internal initialization can be omitted in order to speed up the setup procedure (Ensure, to call initialize() after setting all goals).
        */
        virtual void setGoal(const Eigen::Vector3f& goal, SceneObjectPtr tcp = SceneObjectPtr(), IKSolver::CartesianSelection mode = IKSolver::Position, float tolerancePosition = 5.0f, float toleranceRotation = 3.0f / 180.0f * M_PI, bool performInitialization = true);


        /*! @brief Returns the Jacobian matrix for a given tcp.
            \param tcp The tcp joint that should be considered. By default the tcp joint that is defined in rns in the constructor is used.
            \param mode Allows to include only a subset of the Cartesian coordinates in the Jacobian (e.g., X|Y if the z-component is unimportant)
            \return The Jacobian matrix; The number of rows depends on mode.
             \note{Important} In most cases, DifferentialIK::computeSteps() should be used rather than handling the Jacobian matrix by one self.
            @details
            Please note the convention used to describe the orientation: the scaled axis representation. Orientations in the target vector e (see the example below)
            have to be given as vectors parallel to the rotation axis. Their lengths have to be the rotation angle in radians.
            Given a target pose matrix and the actual tcp pose, the pseudo inverse Jacobian matrix can be used to compute the first Taylor expansion of the IK as follows:
            \code
            // given pose matrices
            Matrix4f target_pose, actual_pose;

            // the error vector
            VectorXd e(6);

            // The translational error is just the vector  between the actual and the target position
            e.segment(0,3) = target_pose(0,3,3,1) - actual_pose(0,3,3,1);

            // For the rotational error, the transformation between the poses has to be calculated and
            // reformulated into the rotation axis and angle. The error is then the rotation axis scaled
            // by the angle in radians.
            Matrix4f orientation = targets_pose * actual_pose.inverse();
            AngleAxis<float> aa(orientation.block<3,3>(0,0));
            e.segment(3,3) = aa.axis()*aa.angle();
            // or
            AngleAxis orientation( target_pose * actual_pose.inverse()   )
            e.block(3,3) = orientation.axis() * orientation.angle();

            // Calculate the IK
            Vector Xd dTheta = dIK->getPseudoInverseJacobianMatrix() * e;
            \endcode
            All this is done within computeSteps(). For more information regarding the differential inverse kinematics,
            see <a href="http://graphics.ucsd.edu/courses/cse169_w05/CSE169_13.ppt">this lecture</a>.
        */
        virtual Eigen::MatrixXf getJacobianMatrix(SceneObjectPtr tcp, IKSolver::CartesianSelection mode);
        virtual Eigen::MatrixXf getJacobianMatrix(SceneObjectPtr tcp);
        virtual Eigen::MatrixXf getJacobianMatrix(IKSolver::CartesianSelection mode);

        /*!
            Computes the complete Jacobian that considers all defined TCPs and goal poses.
        */
        virtual Eigen::MatrixXf getJacobianMatrix();

        /*!
            Computes the complete error vector, considering all TCPs and goals.
        */
        virtual Eigen::VectorXf getError(float stepSize = 1.0f);
        void updateError(Eigen::VectorXf& error, float stepSize = 1.0f);

        /*! @brief Returns the pseudo inverse of the Jacobian matrix for a given tcp of the robot.
         * @see getJacobianMatrix
         * @details The pseudo inverse \f$J^{+}\f$ can be calculated from the Jacobian matrix \f$ J \f$ using the following formula:
         * \f[ J^t \cdot \left( J \cdot J^t \right)^{-1}.\f]. Update: In order to improve stability, we are now using singular value decomposition (SVD).
         */
        virtual Eigen::MatrixXf getPseudoInverseJacobianMatrix(SceneObjectPtr tcp, IKSolver::CartesianSelection mode = IKSolver::All);
        virtual Eigen::MatrixXf getPseudoInverseJacobianMatrix();
        virtual Eigen::MatrixXf getPseudoInverseJacobianMatrix(IKSolver::CartesianSelection mode);

        void updateJacobianMatrix(Eigen::MatrixXf &jac);
        void updateJacobianMatrix(Eigen::MatrixXf &jac, SceneObjectPtr tcp, IKSolver::CartesianSelection mode);
        //Eigen::MatrixXf computePseudoInverseJacobianMatrix(const Eigen::MatrixXf &m);


        /*! @brief Compute a single IK step.
            @param stepSize Controls the amount of error to be reduced in each step: \f$ 0 < \beta \leq 1 \f$
            @return The changes \f$\Delta \theta\f$ in the joint angles.
            \note{Note} This does not affect the joints values of the robot.
        */
        virtual Eigen::VectorXf computeStep(float stepSize = 1.0f);


        /*! @brief Computes the complete inverse kinematics.
            @param stepSize Controls the amount of error to be reduced in each step: \f$ 0 < \beta \leq 1 \f$
            @param maxSteps Maximal numbers of steps.
            @param minChange The minimal change in joint angles (euclidean distance in radians)
            @note{Note}  Sets the node's joint angles automatically.
        */
        virtual bool computeSteps(float stepSize, float minChange, int maxSteps);

        /*! @brief Computes the complete inverse kinematics.
            @param stepSize Controls the amount of error to be reduced in each step: \f$ 0 < \beta \leq 1 \f$
            @param maxSteps Maximal numbers of steps.
            @param minChange The minimal change in joint angles (euclidean distance in radians)
            @note{Note}  Sets the node's joint angles automatically.
        */
        virtual bool solveIK(float stepSize = 0.2f, float minChange = 0.0f, int maxSteps = 50);


        //! Returns the default tcp of the robot.
        virtual RobotNodePtr getDefaultTCP();


        //! Returns the translation error for a given tcp (i.e., the distance to the target).
        virtual float getErrorPosition(SceneObjectPtr tcp = SceneObjectPtr());


        //! Returns the error metric for a given tcp (i.e., the angle between the target pose).
        virtual float getErrorRotation(SceneObjectPtr tcp = SceneObjectPtr());

        /*!
            Enable or disable the check for improvements during computeSteps loop.
            If enabled and the delta to the goal does not gets smaller, the loop is canceled.
            Standard: disabled
        */
        virtual void checkImprovements(bool enable);

        /*!
            If enabled, the Jacobian is computed for [m] while assuming the kinematic definitions are given in [mm].
            Standard: disabled
        */
        void convertModelScalingtoM(bool enable);

        void setVerbose(bool enable);

        /*!
            Returns distance to goal. If multiple goals/TCPs are defined the mean distance is returned.
        */
        virtual float getMeanErrorPosition();

        /*!
            Returns 6D workspace delta that is used for Jacobi calculation.
            Updates the currentDeltaToGoal vector
        */
        virtual void updateDeltaToGoal(Eigen::VectorXf &delta, SceneObjectPtr tcp = SceneObjectPtr());
        virtual void updateDelta(Eigen::VectorXf &delta, const Eigen::Matrix4f& current, const Eigen::Matrix4f& goal, IKSolver::CartesianSelection mode = IKSolver::All);
        /*!
        Returns 6D workspace delta that is used for Jacobi calculation.
        */
        /*!
            Returns 6D workspace delta that is used for Jacobi calculation.
        */
        virtual Eigen::VectorXf getDeltaToGoal(SceneObjectPtr tcp = SceneObjectPtr());
        virtual Eigen::VectorXf getDelta(const Eigen::Matrix4f& current, const Eigen::Matrix4f& goal, IKSolver::CartesianSelection mode = IKSolver::All);


        //! When considering large errors, the translational part can be cut to this length. Set to <= 0 to ignore cutting (standard)
        virtual void setMaxPositionStep(float s);
        virtual bool checkTolerances();

        /*!
            Initializes the internal data structures according to setGoal setup. 
            Usually no need to call this method explicitly, unless performInitialization was not requested in setGoal.
        */
        virtual void initialize();

        virtual void print();
    protected:
        virtual void setNRows();

        float invParam;
        std::vector<SceneObjectPtr> tcp_set;
        RobotNodePtr coordSystem;

        bool checkImprovement; //!< Indicates if the jacobian steps must improve the result, otherwise the loop is canceld.

        std::size_t nRows;
        std::size_t nDoF;

        // need a specialized Eigen allocator here, see http://eigen.tuxfamily.org/dox/TopicStlContainers.html
        std::map<SceneObjectPtr, Eigen::Matrix4f, std::less<SceneObjectPtr>, Eigen::aligned_allocator<std::pair<const SceneObjectPtr, Eigen::Matrix4f> > > targets;
        std::map<SceneObjectPtr, Eigen::MatrixXf, std::less<SceneObjectPtr>, Eigen::aligned_allocator<std::pair<const SceneObjectPtr, Eigen::MatrixXf> > > partJacobians;
        std::map<SceneObjectPtr, IKSolver::CartesianSelection> modes;
        std::map<SceneObjectPtr, float> tolerancePosition;
        std::map<SceneObjectPtr, float> toleranceRotation;

        bool convertMMtoM; // if set, the distances for Jacobian computations are scaled with 1/1000, otherwise the scaling of the model is used (which usually is mm)

        std::vector <RobotNodePtr> nodes;
        std::map< RobotNodePtr, std::vector<RobotNodePtr> > parents;

        Eigen::VectorXf currentError;
        Eigen::MatrixXf currentJacobian;
        Eigen::MatrixXf currentInvJacobian;


        // temporary variables
        Eigen::Matrix4f tmpDeltaOrientation;
        Eigen::AngleAxis<float> tmpDeltaAA;

        Eigen::VectorXf tmpUpdateErrorDelta;
        Eigen::Vector3f tmpUpdateErrorPosition;

        Eigen::MatrixXf tmpUpdateJacobianPosition;
        Eigen::MatrixXf tmpUpdateJacobianOrientation;

        Eigen::VectorXf tmpComputeStepTheta;



        float positionMaxStep;
        bool verbose;

    };

    typedef boost::shared_ptr<DifferentialIK> DifferentialIKPtr;
} // namespace VirtualRobot

#endif // _VirtualRobot_DiffIK_h_
