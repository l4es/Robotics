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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010, 2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_EndEffector_h_
#define _VirtualRobot_EndEffector_h_

#include "../VirtualRobotImportExport.h"



#include <string>
#include <vector>
#include <map>
#include <Eigen/Core>

namespace VirtualRobot
{
    class EndEffectorActor;

    class VIRTUAL_ROBOT_IMPORT_EXPORT EndEffector : public boost::enable_shared_from_this<EndEffector>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        struct ContactInfo
        {
            EndEffectorPtr eef;             // the eef
            EndEffectorActorPtr actor;      // an eef may have multiple actors
            RobotNodePtr robotNode;         // an actor may have multiple robotNodes
            SceneObjectPtr obstacle;
            float distance;
            Eigen::Vector3f contactPointFingerLocal;        // given in coord system of the object
            Eigen::Vector3f contactPointObstacleLocal;      // given in coord system of the object
            Eigen::Vector3f contactPointFingerGlobal;       // given in global coord system
            Eigen::Vector3f contactPointObstacleGlobal;     // given in global coord system

            Eigen::Vector3f approachDirectionGlobal;    // the movement of the contact point while closing the finger (in this direction force can be applied)
        };

        //! We need an Eigen::aligned_allocator here, otherwise access to a std::vector could crash
        typedef std::vector< ContactInfo, Eigen::aligned_allocator<ContactInfo> > ContactInfoVector;

        EndEffector(const std::string& nameString, const std::vector<EndEffectorActorPtr>& actorsVector, const std::vector<RobotNodePtr>& staticPartVector, RobotNodePtr baseNodePtr, RobotNodePtr tcpNodePtr, RobotNodePtr gcpNodePtr = RobotNodePtr(), std::vector< RobotConfigPtr > preshapes = std::vector< RobotConfigPtr >());

        virtual ~EndEffector();
        /*!
            Clones this eef and performs all necessary registrations.
            (Be careful: no need to call registerEEF with newly created eef, since this is already done)
        */
        EndEffectorPtr clone(RobotPtr newRobot);

        std::string getName();


        /*!
            Return name of Base RobotNode.
        */
        std::string getBaseNodeName();


        /*!
            Return name of TCP RobotNode.
        */
        std::string getTcpName();

        RobotNodePtr getTcp();
        RobotNodePtr getBase();

        /*!
            Returns the grasp center point. If it was not defined, the TCP is returned.
        */
        RobotNodePtr getGCP();

        /*!
            Return associated robot
        */
        RobotPtr getRobot();

        CollisionCheckerPtr getCollisionChecker();

        /*!
            Return the type of the associated robot
        */
        std::string getRobotType();

        void getActors(std::vector<EndEffectorActorPtr>& actors);
        void getStatics(std::vector<RobotNodePtr>& statics);

        /*!
            Closes each actor until a joint limit is hit or a collision occurred.
            This method is intended for gripper or hand-like end-effectors.
        */
        ContactInfoVector closeActors(SceneObjectSetPtr obstacles = SceneObjectSetPtr(), float stepSize = 0.02);
        ContactInfoVector closeActors(SceneObjectPtr obstacle, float stepSize = 0.02);

        /*!
            Opens each actor until a joint limit is hit or a collision occurred.
            This method is intended for hand-like end-effectors.
            Note that the same effect can be realized by calling closeActors with a negative step size
        */
        void openActors(SceneObjectSetPtr obstacles = SceneObjectSetPtr(), float stepSize = 0.02);

        /*!
            Build a SceneObjectSet that covers all RobotNodes of this EndEffector.
            \note The set can be used for collision detection, e.g. to check if the eef is in collision with an obstacle.
        */
        SceneObjectSetPtr createSceneObjectSet(CollisionCheckerPtr colChecker = CollisionCheckerPtr());

        /*!
            Construct a robot that consists only of this eef.
            All corresponding robot nodes with visualization and collision models are cloned.
            The resulting robot will have one end effector defined which is identical to this object.
        */
        RobotPtr createEefRobot(const std::string& newRobotType, const std::string& newRobotName, CollisionCheckerPtr collisionChecker = CollisionCheckerPtr());

        /*!
            Register a preshape to this EEF. An exception is thrown if preshape covers joints that are not part of this eef.
            If there is already a preshape with name preshape->getName(), it is replaced.
        */
        void registerPreshape(RobotConfigPtr preshape);

        RobotConfigPtr getPreshape(const std::string& name);

        bool hasPreshape(const std::string& name);

        /*!
            Set joints of this eef to preshape with given name.
            \param name The name of the registered preshape.
            \return false if preshape was not registered, otherwise true
        */
        bool setPreshape(const std::string& name);

        /*!
            Returns vector of all registered preshape names.
        */
        std::vector<std::string> getPreshapes();

        /*!
            Check, if node is part of this eef.
        */
        bool hasNode(RobotNodePtr node);

        void print();

        /*!
            Return current configuration as robot config.
        */
        RobotConfigPtr getConfiguration();

        //! Return all associated robot nodes
        std::vector< RobotNodePtr > getAllNodes();

        //! Returns true, if nodes (only name strings are checked)  are sufficient for building this eef
        bool nodesSufficient(std::vector<RobotNodePtr> nodes) const;

        /*!
            returns an approximation about the length of this eef.
        */
        float getApproximatedLength();

        /*!
            Creates an XML string of this EEF.
        */
        virtual std::string toXML(int ident = 1);

    private:
        std::string name;
        std::vector<EndEffectorActorPtr> actors;
        std::vector<RobotNodePtr> statics;
        std::map< std::string, RobotConfigPtr > preshapes;
        RobotNodePtr baseNode;
        RobotNodePtr tcpNode;
        RobotNodePtr gcpNode;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_EndEffector_h_
