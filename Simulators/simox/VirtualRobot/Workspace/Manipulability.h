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
#ifndef _VirtualRobot_Manipulability_h_
#define _VirtualRobot_Manipulability_h_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Workspace/WorkspaceRepresentation.h>
#include <VirtualRobot/IK/PoseQualityMeasurement.h>
#include <VirtualRobot/Grasping/GraspSet.h>


namespace VirtualRobot
{

    /*!
            This class represents an approximation of the distribution of a kinematic chain's manipulability.
            Therefore, the Cartesian space (6D) is voxelized. Each voxel holds a quality value, approximatively
            representing the maximum manipulability that can be achieved at the given pose.
            The discretized manipulability data can be written to and loaded from binary files.

            The manipulability is linked to a base coordinate system which is defined by a robot joint.
            This base system is used to align the data when the robot is moving.
            I.e. think of an arm of a humanoid where the manipulability data is linked to the shoulder.
            When the torso moves, the manipulability also changes it's position according to the position of the shoulder.

            For buildup different manipulability measures can be incorporated (e.g. Yoshikawa's manipulability measure or extensions to this approach).
            \see PoseQualityMeasurement
            \see PoseQualityManipulability
            \see PoseQualityExtendedManipulability
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT Manipulability : public WorkspaceRepresentation, public boost::enable_shared_from_this<Manipulability>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Manipulability(RobotPtr robot);


        struct ManipulabiliyGrasp
        {
            GraspPtr grasp;
            float manipulability;
            bool operator<(const ManipulabiliyGrasp& rhs) const
            {
                return manipulability < rhs.manipulability;
            }
        };

        /*!
            Performs a manipulability analysis of grasps at the current position of the object.
            \param grasps The grasps that should be analyzed.
            \param object The grasps are supposed to to be applied to the object at its current global pose.
            \return A vector of all grasps with manipulability>0 in sorted order (starting with the grasp with highest manipulability)
        */
        std::vector< ManipulabiliyGrasp > analyseGrasps(GraspSetPtr grasps, ManipulationObjectPtr object);
        ManipulabiliyGrasp analyseGrasp(GraspPtr grasp, ManipulationObjectPtr object);

        /*!
            The manipulability measure can be defined here
        */
        void setManipulabilityMeasure(PoseQualityMeasurementPtr m);

        /*!
            Returns the name of the manipulability measure.
        */
        std::string getMeasureName() const;

        /*!
            Returns true, if manipulability measure considers joint limits.
        */
        bool consideringJointLimits() const;

        /*!
            All entries are scaled (during construction) according to this value.
            Higher manipulability values will result in an entry of 255
        */
        void setMaxManipulability(float maximalManip);

        float getMaxManipulability();

        /*!
            Returns the maximal manipulability that can approximatively be achieved at globalPose.
            Therefore, the entry of the discretized manipulability data is used and projected to [0,maxManip]
        */
        float getManipulabilityAtPose(const Eigen::Matrix4f& globalPose);

        /*!
            Initializes the consideration of self distances (off by default).
            If enabled, the distance vector between static and dynamic rns is computed for every random pose and
            the quality measure is penalized according to that vector.
            \param staticModel The static (not moving) part of the robot (e.g. torso and head)
            \param staticModel The dynamic (moving) part of the robot. Recommending to use the end effector instead of the complete arm, since the upperarm usually has a low distance to the rest of the body.
        */
        void initSelfDistanceCheck(RobotNodeSetPtr staticModel, RobotNodeSetPtr dynamicModel);

        /*!
            Do a test run in order to get an idea of the covered workspace extends and the maximum manipulability.
            The resulting values can be used to initialize this object.
            Values are slightly scaled.
            \param nodeSet These nodes should be considered.
            \param steps How many steps should be performed (use a large value e.g. 1000)
            \param storeMinBounds The achieved minimum values are stored here.
            \param storeMaxBounds The achieved maximum values are stored here.
            \param storeMaxManipulability The maximal achieved manipulability is stored here.
            \param baseNode The base node.
            \param tcpNode The tcp.

            \see WorkspaceRepresentation::initialize()
        */
        virtual bool checkForParameters(RobotNodeSetPtr nodeSet,
                                        float steps,
                                        float storeMinBounds[6],
                                        float storeMaxBounds[6],
                                        float& storeMaxManipulability,
                                        RobotNodePtr baseNode = RobotNodePtr(),
                                        RobotNodePtr tcpNode = RobotNodePtr());


        /*!
            smooth the data
        */
        bool smooth(unsigned int minNeighbors = 1);


        /*!
            Returns all reachable grasps that can be applied at the current position of object.
        */
        GraspSetPtr getReachableGrasps(GraspSetPtr grasps, ManipulationObjectPtr object);

        /*!
            Access self distance configuration.
        */
        void getSelfDistConfig(bool& storeConsiderSelfDist, RobotNodeSetPtr& storeStatic, RobotNodeSetPtr& storeDynamic);

        /*!
            Creates a deep copy of this data structure. A ManipulabilityPtr is returned.
        */
        virtual WorkspaceRepresentationPtr clone();
    protected:

        virtual bool customLoad(std::ifstream& file);
        virtual bool customSave(std::ofstream& file);
        virtual void customPrint();

        virtual void customInitialize();

        bool customStringRead(std::ifstream& file, std::string& res);



        float getCurrentManipulability();
        void addPose(const Eigen::Matrix4f& p);
        PoseQualityMeasurementPtr measure;

        float maxManip;

        std::string measureName; // if file was loaded we store the name of the manipulability measure here
        bool considerJL;

        bool considerSelfDist;

        RobotNodeSetPtr selfDistStatic;
        RobotNodeSetPtr selfDistDynamic;

    };

    typedef boost::shared_ptr<Manipulability> ManipulabilityPtr;

} // namespace VirtualRobot

#endif // _VirtualRobot_Manipulability_h_

