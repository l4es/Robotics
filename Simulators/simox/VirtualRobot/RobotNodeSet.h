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
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_RobotNodeSet_h_
#define _VirtualRobot_RobotNodeSet_h_

#include "VirtualRobotImportExport.h"
#include "Nodes/RobotNode.h"
#include "SceneObjectSet.h"
#include <string>
#include <vector>

namespace VirtualRobot
{
    class Robot;

    /*!
        A RobotNodeSet is a sub-set of RobotNodes of a Robot.
        Additionally to the list of RobotNodes, a RobotNodeSet holds information about
        - the kinematic root (the topmost RobotNode of the Robot's kinematic tree that has to be updated in order to update all covered RobotNodes)
        - the Tool Center point (TCP)
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT RobotNodeSet : public SceneObjectSet, public boost::enable_shared_from_this<RobotNodeSet>
    {
    public:
        typedef std::vector<RobotNodePtr> NodeContainerT;
        typedef NodeContainerT::iterator  NodeContainerIterT;

        friend class RobotFactory;

        /*!
            Use this method to create and fully initialize an instance of RobotNodeSet.
        */
        static RobotNodeSetPtr createRobotNodeSet(RobotPtr robot, const std::string& name, const std::vector< std::string >& robotNodeNames, const std::string& kinematicRootName = "", const std::string& tcpName = "", bool registerToRobot = false);
        /*!
            Use this method to create and fully initialize an instance of RobotNodeSet.
        */
        static RobotNodeSetPtr createRobotNodeSet(RobotPtr robot, const std::string& name, const std::vector< RobotNodePtr >& robotNodes, const RobotNodePtr kinematicRoot = RobotNodePtr(), const RobotNodePtr tcp = RobotNodePtr(), bool registerToRobot = false);

        /*!
            Registers a copy of this node set with the given robot
         */
        RobotNodeSetPtr clone(RobotPtr newRobot, const RobotNodePtr newKinematicRoot = RobotNodePtr());

        bool hasRobotNode(RobotNodePtr robotNode) const;

        /*!
            Returns all nodes.
        */
        const std::vector< RobotNodePtr > getAllRobotNodes() const;

        /*!
            Returns the topmost node of the robot's kinematic tree to be used for updating all members of this set.
            This node is usually defined in the RobotNodeSet's XML definition.
        */
        RobotNodePtr getKinematicRoot() const;

        void setKinematicRoot(RobotNodePtr robotNode);

        /*!
            Returns the TCP.
        */
        RobotNodePtr getTCP() const;

        //! Print out some information.
        void print() const;

        /*!
            The number of associated robot nodes.
        */
        virtual unsigned int getSize() const;

        std::vector<float> getJointValues() const;
        void getJointValues(std::vector<float>& fillVector) const;
        void getJointValues(Eigen::VectorXf& fillVector) const;
        void getJointValues(RobotConfigPtr fillVector) const;

        /*!
        Cut
        */
        void respectJointLimits(std::vector<float>& jointValues) const;
        void respectJointLimits(Eigen::VectorXf& jointValues) const;

        /*!
            Checks if the jointValues are within the current joint limits.
            \param jointValues A vector of correct size.
            \param verbose Print information if joint limits are violated.
            \return True when all given joint values are within joint limits.
        */
        bool checkJointLimits(std::vector<float>& jointValues, bool verbose = false) const;
        bool checkJointLimits(Eigen::VectorXf& jointValues, bool verbose = false) const;

        /*!
            Set joint values [rad].
            The subpart of the robot, defined by the start joint (kinematicRoot) of rns, is updated to apply the new joint values.
            \param jointValues A vector with joint values, size must be equal to number of joints in this RobotNodeSet.
        */
        void setJointValues(const std::vector<float>& jointValues);
        /*!
            Set joint values [rad].
            The subpart of the robot, defined by the start joint (kinematicRoot) of rns, is updated to apply the new joint values.
            \param jointValues A vector with joint values, size must be equal to number of joints in this RobotNodeSet.
        */
        void setJointValues(const Eigen::VectorXf& jointValues);

        /*!
            Set joints that are within the given RobotConfig. Joints of this NodeSet that are not stored in jointValues remain untouched.
        */
        void setJointValues(const RobotConfigPtr jointValues);

        RobotNodePtr& operator[](int i);

        RobotNodePtr& getNode(int i);

        // implement container interface for easy access
        inline NodeContainerIterT begin()
        {
            return robotNodes.begin();
        }
        inline NodeContainerIterT end()
        {
            return robotNodes.end();
        }

        RobotPtr getRobot();

        //CollisionCheckerPtr getCollisionChecker();

        /*!
            Checks if this set of robot nodes form a valid kinematic chain.
        */
        bool isKinematicChain();

        KinematicChainPtr toKinematicChain();

        /*!
          Convenient method for highlighting the visualization of this node.
          It is automatically checked whether the collision model or the full model is part of the visualization.
          \param visualization The visualization for which the highlighting should be performed.
          \param enable Set or unset highlighting.
        */
        virtual void highlight(VisualizationPtr visualization, bool enable);

        /*!
            get number of faces (i.e. triangles) of this object
            \p collisionModel Indicates weather the faces of the collision model or the full model should be returned.
        */
        virtual int getNumFaces(bool collisionModel = false);

        /*!
            Compute an upper bound of the extension of the kinematic chain formed by this RobotNodeSet.
            This is done by summing the distances between all succeeding RobotNodes of this set.
        */
        float getMaximumExtension();

        /*!
            Return center of mass of this node set.
        */
        Eigen::Vector3f getCoM();

        /*!
            Return accumulated mass of this node set.
        */
        float getMass();

        //! Returns true, if nodes (only name strings are checked)  are sufficient for building this rns
        bool nodesSufficient(std::vector<RobotNodePtr> nodes) const;


        virtual std::string toXML(int tabs);

        //! this is forbidden for RobotNodeSets, a call will throw an exception
        virtual bool addSceneObject(SceneObjectPtr sceneObject);
        //! this is forbidden for RobotNodeSets, a call will throw an exception
        virtual bool addSceneObjects(SceneObjectSetPtr sceneObjectSet);
        //! this is forbidden for RobotNodeSets, a call will throw an exception
        virtual bool addSceneObjects(RobotNodeSetPtr robotNodeSet);
        //! this is forbidden for RobotNodeSets, a call will throw an exception
        virtual bool addSceneObjects(std::vector<RobotNodePtr> robotNodes);
        //! this is forbidden for RobotNodeSets, a call will throw an exception
        virtual bool removeSceneObject(SceneObjectPtr sceneObject);

    protected:
        /*!
         * Tests if the given robot node is a valid kinematic root
         */
        bool isKinematicRoot(RobotNodePtr robotNode);

    protected:
        /*!
            Initialize this set with a vector of RobotNodes.
            \param name A name
            \param robot A robot
            \param robotNodes A set of robot nodes.
            \param kinematicRoot    This specifies the first node of the robot's kinematic tree to be used for updating all members of this set.
                                    kinematicRoot does not have to be a node of this set.
                                    If not given, the first entry of robotNodes is used.
            \param tcp The tcp.
        */
        RobotNodeSet(const std::string& name, RobotWeakPtr robot, const std::vector< RobotNodePtr >& robotNodes, const RobotNodePtr kinematicRoot = RobotNodePtr(), const RobotNodePtr tcp = RobotNodePtr());
        NodeContainerT robotNodes;
        RobotWeakPtr robot;

        RobotNodePtr kinematicRoot;
        RobotNodePtr tcp;

        //bool kinematicRootIsRobotRoot;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_RobotNodeSet_h_
