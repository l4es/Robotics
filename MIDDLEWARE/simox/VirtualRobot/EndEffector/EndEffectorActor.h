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
* @copyright  2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_EndEffectorActor_h_
#define _VirtualRobot_EndEffectorActor_h_

#include "../VirtualRobotImportExport.h"
#include "EndEffector.h"



#include <string>
#include <vector>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT EndEffectorActor : public boost::enable_shared_from_this<EndEffectorActor>
    {
    public:
        enum CollisionMode
        {
            eNone = 0,
            eActors = 1,
            eStatic = 2,
            eAll = 3
        };

        struct ActorDefinition
        {
            CollisionMode colMode;
            RobotNodePtr robotNode;
            float directionAndSpeed;
        };

        EndEffectorActor(const std::string& name, const std::vector< ActorDefinition >& a, CollisionCheckerPtr colChecker = CollisionCheckerPtr());

        std::vector<ActorDefinition> getDefinition();

        /*!
            Clones the EndEffectorActor with respect to the given robot. Note that unlike the clone-methods of
            RobotNodeSet and EndEffector, this method does not register anything with the robot.
        */
        EndEffectorActorPtr clone(RobotPtr newRobot);

        std::string getName();

        /*!
            Changes the value of all joints belonging to the actor by angle.
            Returns true if all joints hit their limit.
        */
        bool moveActor(float angle = 0.02);

        /*!
            Changes the value of all joints belonging to the actor by angle while checking every RobotNode for collisions with eef and obstacles.
            \param eef The End effector
            \param storeContacts In case collisions are detected, the corresponding contact info is stored here.
            \param obstacles Check collisions with these obstacles.
            \param angle How far should the eef actor move [rad]
            Returns true if all joints do either hit their limit or are in collision, e.g. the actor cannot be moved any further.
        */
        bool moveActorCheckCollision(EndEffectorPtr eef, EndEffector::ContactInfoVector& storeContacts, SceneObjectSetPtr obstacles = SceneObjectSetPtr(), float angle = 0.02);

        /*!
            Checks if the actor collides with one of the given obstacles
        */
        bool isColliding(SceneObjectSetPtr obstacles, CollisionMode checkColMode = EndEffectorActor::eAll);
        bool isColliding(EndEffectorPtr eef, SceneObjectSetPtr obstacles, EndEffector::ContactInfoVector& storeContacts, CollisionMode checkColMode = EndEffectorActor::eAll);

        /*!
            Checks if the actor collides with the given obstacle.
            \p checkColMode If set, the collisionMode of the actor's robotNodes is checked against it (e.g. to avoid collision checks with the static part of the eef)
        */
        bool isColliding(SceneObjectPtr obstacle, CollisionMode checkColMode = EndEffectorActor::eAll);
        bool isColliding(EndEffectorPtr eef, SceneObjectPtr obstacle, EndEffector::ContactInfoVector& storeContacts, CollisionMode checkColMode = EndEffectorActor::eAll);

        /*!
            Checks if the actor collides with a given second actor
        */
        bool isColliding(EndEffectorActorPtr obstacle);
        bool isColliding(EndEffectorPtr eef, EndEffectorActorPtr obstacle, EndEffector::ContactInfoVector& storeContacts);

        /*!
            Checks if the actor collides with a given end effector
        */
        bool isColliding(EndEffectorPtr obstacle);
        bool isColliding(EndEffectorPtr eef, EndEffectorPtr obstacle, EndEffector::ContactInfoVector& storeContacts);

        std::vector< RobotNodePtr > getRobotNodes();

        void print();

        /*!
            Check, if node is part of this actor.
        */
        bool hasNode(RobotNodePtr node);

        //! Returns true, if nodes (only name strings are checked) are sufficient for building this eef
        bool nodesSufficient(std::vector<RobotNodePtr> nodes) const;

        /*!
            returns an approximation about the length of this eef.
        */
        float getApproximatedLength();

        RobotConfigPtr getConfiguration();

        virtual std::string toXML(int ident = 1);

    private:

        std::string name;
        std::vector<ActorDefinition> actors;

        CollisionCheckerPtr colChecker;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_EndEffectorActor_h_
