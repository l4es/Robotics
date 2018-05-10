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
* @package    Saba
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _Saba_GraspRrt_h
#define _Saba_GraspRrt_h

#include "../Saba.h"
#include "../CSpace/CSpaceSampled.h"
#include "../CSpace/CSpacePath.h"
#include "../CSpace/CSpaceNode.h"
#include "../ApproachDiscretization.h"
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/IK/IKSolver.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/Obstacle.h>
#include "Rrt.h"

namespace Saba
{

    /*!
     *
     *
    * A planner that combines the search for a feasible grasping pose with RRT-based motion planning.
    * No predefined grasps are needed, a feasible grasp is searched during the planning process.
    * Makes use of the DifferntialIK methods to generate potential approach movements.
    * A GraspQualityMeasure object is needed in order to evaluate generated grasping hypotheses.
    *
    * Related publication: "Integrated Grasp and Motion Planning", N. Vahrenkamp, M. Do, T. Asfour, R. Dillmann, ICRA 2010
    *
    */
    class SABA_IMPORT_EXPORT GraspRrt : public Rrt
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
            \param cspace   The C-Space that should be used for collision detection.
                            The RobotNodeSet, that was used to define the cspace, is internally used to generate approach movements.
            \param eef The EndEffector that should be used for grasping.
            \param object The object to be grasped
            \param measure A measure object that should be used for evaluating the quality of generated grasping hypotheses.
            \param graspCollisionObjects These (optional) objects are considered as obstacles when closing the hand. The targetObject is handled explicitly and need not be part of these object set.
            \param probabGraspHypothesis The probability that a grasping hypothesis is generated during a loop.
            \param graspQualityMinScore The minimum score that must be achieved for a valid grasp.

            @see GraspStudio::GraspQualityMeasureWrenchSpace
        */
        GraspRrt(CSpaceSampledPtr cspace,
                 VirtualRobot::EndEffectorPtr eef,
                 VirtualRobot::ObstaclePtr object,
                 VirtualRobot::BasicGraspQualityMeasurePtr measure,
                 VirtualRobot::SceneObjectSetPtr graspCollisionObjects = VirtualRobot::SceneObjectSetPtr(),
                 float probabGraspHypothesis = 0.1f,
                 float graspQualityMinScore = 0.01f);

        virtual ~GraspRrt();

        /*!
            do the planning (blocking method)
            \return true if solution was found, otherwise false
        */
        virtual bool plan(bool bQuiet = false);


        virtual void printConfig(bool printOnlyParams = false);
        virtual bool setStart(const Eigen::VectorXf& c);

        //! This is not allowed here, since we sample goal configurations during planning: If called an exception is thrown
        virtual bool setGoal(const Eigen::VectorXf& c);

        //! reset the planner
        virtual void reset();

        /*!
            Returns the internal representation of the pose sphere.
            The pose sphere is used to encode approach directions.
        */
        ApproachDiscretizationPtr getPoseRelationSphere();

        struct GraspInfo
        {
            Eigen::Matrix4f handToObjectTransform;
            int rrtNodeId;
            float graspScore;
            float distanceToObject;
            VirtualRobot::EndEffector::ContactInfoVector contacts;
        };

        typedef std::vector< GraspInfo, Eigen::aligned_allocator<GraspInfo> > GraspInfoVector;


        //! Stores all found grasps to given vector (thread safe)
        void getGraspInfoResult(GraspInfoVector& vStoreGraspInfo);

        //! Returns a specific grasp result (thread safe)
        bool getGraspInfoResult(int index, GraspInfo& storeGraspInfo);

        //! Returns number of (currently) found grasps (thread safe)
        int getNrOfGraspInfoResults();

        /*!
            Creates a solution with last entry cspaceNodeIndex.
            You are responsible for deleting.
        */
        //solution* buildSolution(int cspaceNodeIndex);

        struct PlanningPerformance
        {
            float setupTimeMS;
            float planningTimeMS; // == RRTTime + moveTime + graspScoreTime
            float moveTowardGraspPosesTimeMS; // accumulated time
            int numberOfMovementsTowardGraspPose;
            float scoreGraspTimeMS; // accumulated time
            int numberOfGraspScorings;
            float rrtTimeMS;
        };

        PlanningPerformance getPerformanceMeasurement();
        void printPerformanceResults();

        /*!
            This method generates a grasping pose on the object's surface.
            The z-axis of the GCP's coordinate system is aligned with the normal.
            We assume that the z axis specifies the favorite approach movement of the EEF.
        */
        bool calculateGlobalGraspPose(const Eigen::VectorXf& c, Eigen::Matrix4f& storeGoal);

        enum MoveArmResult
        {
            eMovedOneStep,              // arm movement was successful, but goal not reached yet
            eGoalReached,               // goal reached
            eCollision_Environment,     // arm or hand collided with environment
            eGraspablePoseReached,      // a position which allows applying a feasible grasp was reached
            eJointBoundaryViolation,    // joint limits were violated
            eTrapped,                   // trapped in local minima
            eError,                     // unexpected error
            eFatalError                 // unexpected fatal error
        };

        /*!
            Computes a step towards goalPose.
            \param currentPose The current pose
            \param goalPose The pose in global coord system.
            \param storeCSpaceConf The result is stored here.
            \return The resulting state.
        */
        MoveArmResult createWorkSpaceSamplingStep(const Eigen::Matrix4f& currentPose, const Eigen::Matrix4f& goalPose, Eigen::VectorXf& storeCSpaceConf);

        /*!
            Can be used for custom initialization. Usually this method is automatically called at the beginning of a planning query.
        */
        virtual bool init();

		/*! By default min 3 contacts are needed, this number can be adjusted here.
		*/
		void setMinGraspContacts(int nr);
    protected:

        bool doPlanningCycle();

        /*!
            For all nodes in tree, that form the path from startId to endId processNode is called.
            startId is not processed.
        */
        bool processNodes(unsigned int startId, unsigned int endId);

        //! Compute tcp (gcp) pose and update internal map and pose relation sphere
        bool processNode(CSpaceNodePtr n);

        VirtualRobot::ObstaclePtr targetObject;
        VirtualRobot::RobotNodeSetPtr rns;
        VirtualRobot::EndEffectorPtr eef;
        VirtualRobot::RobotPtr robot;

        std::map < VirtualRobot::GraspPtr, Saba::CSpaceNodePtr > graspNodeMapping;
        std::vector< Eigen::VectorXf > ikSolutions;



        PlanningPerformance performanceMeasure;

		int minGraspContacts;


        MoveArmResult connectRandomGraspPositionJacobian();

        Rrt::ExtensionResult connectComplete();

        float workSpaceSamplingCount;

        VirtualRobot::ObstaclePtr gcpOject; //<! This object is used to determine the distances to the targetObject

        MoveArmResult moveTowardsGoal(CSpaceNodePtr startNode, const Eigen::Matrix4f& targetPose, int nMaxLoops);
        MoveArmResult createWorkSpaceSamplingStep(const Eigen::Matrix4f& goalPose, CSpaceNodePtr extendNode, Eigen::VectorXf& storeCSpaceConf);

        /*!
            Limit the step that is defined with p, so that the translational and rotational stepsize is limited by cartSamplingOriStepSize and cartSamplingPosStepSize
        */
        void limitWorkspaceStep(Eigen::Matrix4f& p);

        /*!
            Current pose of RNS is moved.
            \param deltaPose Consists of the 3x3 rotation delta R and the 3 dim translational delta T in homogeneous notation. R and T must be given in global coordinate system.
            \param storeCSpaceConf The result is stored here.
        */
        MoveArmResult moveArmDiffKin(const Eigen::Matrix4f& deltaPose, Eigen::VectorXf& storeCSpaceConf);


        // Computes the grasp score for given configuration
        // nId is needed, to store the id of the rrt node which holds pConfig
        float calculateGraspScore(const Eigen::VectorXf& c, int nId = -1, bool bStoreGraspInfoOnSuccess = false);


        VirtualRobot::SceneObjectSetPtr graspCollisionObjects; //!< These objects are considered as obstacles when closing the hand. The targetObject is handled explicitly and must not be part of these object set.

        virtual Rrt::ExtensionResult connectComplete(Eigen::VectorXf& c, CSpaceTreePtr tree, int& storeLastAddedID);

        void printGraspInfo(GraspInfo& GrInfo);

        MoveArmResult extendGraspStatus;
        float cartSamplingOriStepSize, cartSamplingPosStepSize;


        GraspInfoVector grasps;

        ApproachDiscretizationPtr poseSphere;

        VirtualRobot::DifferentialIKPtr diffIK;

        Eigen::Vector3f targetObjectPosition; // global pose of manipulation object
        float tryGraspsDistance2; // TCP<->object distance in which grasp movements are tried (squared value)

        //bool m_bEndlessMode;

        int colChecksOverall;
        bool foundSolution;
        boost::mutex graspInfoMutex;

        bool verbose;

        std::map<CSpaceNodePtr, Eigen::Matrix4f, std::less<CSpaceNodePtr>,
            Eigen::aligned_allocator<std::pair<const CSpaceNodePtr, Eigen::Matrix4f> > > mapConfigTcp; //!< Store the tcp (gcp) poses that correspond with the config

        // grasp measurement
        VirtualRobot::BasicGraspQualityMeasurePtr graspQualityMeasure;
        float probabGraspHypothesis;
        float graspQualityMinScore;

        bool plannerInitialized;

    };

}

#endif // _GraspRRTPlanner_h_

