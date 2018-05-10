/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @author     Manfred Kroehnert
* @copyright  2011 Manfred Kroehnert
*/

#include "EndEffectorActor.h"
#include "../VirtualRobotException.h"
#include "../Nodes/RobotNode.h"
#include "../Robot.h"
#include "../RobotConfig.h"
#include "../SceneObjectSet.h"
#include "../CollisionDetection/CollisionChecker.h"
#include "EndEffector.h"
#include "../SceneObjectSet.h"


namespace VirtualRobot
{

    EndEffectorActor::EndEffectorActor(const std::string& name, const std::vector< ActorDefinition >& a, CollisionCheckerPtr colChecker) :
        name(name),
        actors(a)
    {
        this->colChecker = colChecker;

        if (!this->colChecker)
        {
            this->colChecker = CollisionChecker::getGlobalCollisionChecker();
        }
    }

    std::vector<EndEffectorActor::ActorDefinition> EndEffectorActor::getDefinition()
    {
        return actors;
    }

    EndEffectorActorPtr EndEffectorActor::clone(RobotPtr newRobot)
    {
        if (!newRobot)
        {
            VR_ERROR << "Attempting to clone EndEffectorActor for invalid robot";
            return EndEffectorActorPtr();
        }

        std::vector<ActorDefinition> newDef;

        for (unsigned int i = 0; i < actors.size(); i++)
        {
            ActorDefinition a;
            a.colMode = actors[i].colMode;
            a.directionAndSpeed = actors[i].directionAndSpeed;
            a.robotNode = newRobot->getRobotNode(actors[i].robotNode->getName());
            newDef.push_back(a);
        }

        return EndEffectorActorPtr(new EndEffectorActor(name, newDef, newRobot->getCollisionChecker()));
    }

    std::string EndEffectorActor::getName()
    {
        return name;
    }

    bool EndEffectorActor::moveActor(float angle)
    {
        if (actors.size() == 0)
        {
            return true;
        }

        RobotPtr robot = actors[0].robotNode->getRobot();
        VR_ASSERT(robot);
        bool res = true;

        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            float v = n->robotNode->getJointValue() + angle * n->directionAndSpeed;

            if (v <= n->robotNode->getJointLimitHi() && v >= n->robotNode->getJointLimitLo())
            {
                robot->setJointValue(n->robotNode, v);
                //n->robotNode->setJointValue(v);
                res = false;
            }
        }

        return res;
    }

    bool EndEffectorActor::moveActorCheckCollision(EndEffectorPtr eef, EndEffector::ContactInfoVector& storeContacts, SceneObjectSetPtr obstacles /*= SceneObjectSetPtr()*/, float angle /*= 0.02*/)
    {
        VR_ASSERT(eef);
        RobotPtr robot = eef->getRobot();
        VR_ASSERT(robot);
        bool res = true;
        std::vector<EndEffectorActorPtr> eefActors;
        eef->getActors(eefActors);
        std::vector<RobotNodePtr> eefStatic;
        eef->getStatics(eefStatic);
        EndEffector::ContactInfoVector newContacts;

        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            float oldV =  n->robotNode->getJointValue();
            float v = oldV + angle * n->directionAndSpeed;

            if (v <= n->robotNode->getJointLimitHi() && v >= n->robotNode->getJointLimitLo())
            {
                robot->setJointValue(n->robotNode, v);
                //n->robotNode->setJointValue(v);

                // check collision
                bool collision = false;

                // obstacles (store contacts)
                if ((/*n->colMode!=eNone &&*/ obstacles && isColliding(eef, obstacles, newContacts)))
                {
                    collision = true;
                }

                // actors (don't store contacts)
                if (!collision)
                {
                    for (std::vector<EndEffectorActorPtr>::iterator a = eefActors.begin(); a != eefActors.end(); a++)
                    {
                        // Don't check for collisions with the actor itself (don't store contacts)
                        if (((*a)->getName() != name) && isColliding(*a))   //isColliding(eef,*a,newContacts) )
                        {
                            collision = true;
                        }
                    }
                }

                // static (don't store contacts)
                if (!collision)
                {
                    for (std::vector<RobotNodePtr>::iterator node = eefStatic.begin(); node != eefStatic.end(); node++)
                    {
                        SceneObjectPtr so = boost::dynamic_pointer_cast<SceneObject>(*node);

                        //(don't store contacts)
                        //if( isColliding(eef,so,newContacts,eStatic) )
                        if (isColliding(so, eStatic))
                        {
                            collision = true;
                        }
                    }
                }

                if (!collision)
                {
                    res = false;
                }
                else
                {
                    // reset last position
                    //n->robotNode->setJointValue(oldV);
                    robot->setJointValue(n->robotNode, oldV);
                }
            }
        }

        // update contacts
        for (size_t i = 0; i < newContacts.size(); i++)
        {
            // check for double entries (this may happen since we move all actors to the end and may detecting contacts multiple times)
            bool doubleEntry = false;

            for (size_t j = 0; j < storeContacts.size(); j++)
            {
                if (storeContacts[j].robotNode == newContacts[i].robotNode && storeContacts[j].obstacle == newContacts[i].obstacle)
                {
                    doubleEntry = true;
                    break;
                }
            }

            if (!doubleEntry)
            {
                int id1, id2;
                newContacts[i].distance = colChecker->calculateDistance(newContacts[i].robotNode->getCollisionModel(), newContacts[i].obstacle->getCollisionModel(), newContacts[i].contactPointFingerGlobal, newContacts[i].contactPointObstacleGlobal, &id1, &id2);
                newContacts[i].contactPointFingerLocal = newContacts[i].obstacle->toLocalCoordinateSystemVec(newContacts[i].contactPointFingerGlobal);
                newContacts[i].contactPointObstacleLocal = newContacts[i].obstacle->toLocalCoordinateSystemVec(newContacts[i].contactPointObstacleGlobal);

                // compute approach direction
                // todo: this could be done more elegantly (Jacobian)
                RobotConfigPtr config = getConfiguration();
                Eigen::Vector3f contGlobal1 = newContacts[i].contactPointFingerGlobal;
                Eigen::Vector3f contFinger = newContacts[i].robotNode->toLocalCoordinateSystemVec(contGlobal1);
                this->moveActor(angle);
                Eigen::Vector3f contGlobal2 = newContacts[i].robotNode->toGlobalCoordinateSystemVec(contFinger);
                newContacts[i].approachDirectionGlobal = contGlobal2 - contGlobal1;
                newContacts[i].approachDirectionGlobal.normalize();
                robot->setJointValues(config);

                storeContacts.push_back(newContacts[i]);
            }
        }

        return res;
    }


    bool EndEffectorActor::isColliding(EndEffectorPtr eef, SceneObjectSetPtr obstacles, EndEffector::ContactInfoVector& storeContacts, CollisionMode checkColMode)
    {
        std::vector<SceneObjectPtr> colModels = obstacles->getSceneObjects();
        //Eigen::Vector3f contact;
        bool col = false;

        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            for (std::vector<SceneObjectPtr>::iterator o = colModels.begin(); o != colModels.end(); o++)
            {

                if ((n->colMode & checkColMode) &&
                    ((*o)->getCollisionModel()) &&
                    n->robotNode->getCollisionModel() &&
                    colChecker->checkCollision(n->robotNode->getCollisionModel(), (*o)->getCollisionModel()))
                {

                    col = true;
                    // create contact info
                    EndEffector::ContactInfo ci;
                    ci.eef = eef;
                    ci.actor = shared_from_this();
                    ci.robotNode = n->robotNode;
                    ci.obstacle = *o;

                    // todo: maybe not needed here: we are in collision, distance makes no sense...
                    // later the distance is calculated anyway (with slightly opened actors)
                    int id1, id2;
                    ci.distance = colChecker->calculateDistance(ci.robotNode->getCollisionModel(), ci.obstacle->getCollisionModel(), ci.contactPointFingerGlobal, ci.contactPointObstacleGlobal, &id1, &id2);
                    ci.contactPointFingerLocal = ci.obstacle->toLocalCoordinateSystemVec(ci.contactPointFingerGlobal);
                    ci.contactPointObstacleLocal = ci.obstacle->toLocalCoordinateSystemVec(ci.contactPointObstacleGlobal);

                    storeContacts.push_back(ci);
                }
            }
        }

        return col;
    }

    bool EndEffectorActor::isColliding(SceneObjectSetPtr obstacles,  CollisionMode checkColMode)
    {
        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            if ((n->colMode & checkColMode) && n->robotNode->getCollisionModel() && colChecker->checkCollision(n->robotNode->getCollisionModel(), obstacles))
            {
                return true;
            }
        }

        return false;
    }

    bool EndEffectorActor::isColliding(SceneObjectPtr obstacle, CollisionMode checkColMode)
    {
        if (!obstacle || !obstacle->getCollisionModel())
        {
            return false;
        }

        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            if ((n->colMode & checkColMode) && n->robotNode->getCollisionModel() && colChecker->checkCollision(n->robotNode->getCollisionModel(), obstacle->getCollisionModel()))
            {
                return true;
            }
        }

        return false;
    }

    bool EndEffectorActor::isColliding(EndEffectorActorPtr obstacle)
    {
        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            SceneObjectPtr so = boost::dynamic_pointer_cast<SceneObject>(n->robotNode);

            if ((n->colMode & eActors) && obstacle->isColliding(so))
            {
                return true;
            }
        }

        return false;
    }

    bool EndEffectorActor::isColliding(EndEffectorPtr obstacle)
    {
        std::vector<EndEffectorActorPtr> obstacleActors;
        obstacle->getActors(obstacleActors);

        std::vector<RobotNodePtr> obstacleStatics;
        obstacle->getStatics(obstacleStatics);

        for (std::vector<EndEffectorActorPtr>::iterator actor = obstacleActors.begin(); actor != obstacleActors.end(); actor++)
        {
            // Don't check for collisions with the actor itself
            if (((*actor)->getName() != name) && isColliding(*actor))
            {
                return true;
            }
        }

        for (std::vector<RobotNodePtr>::iterator node = obstacleStatics.begin(); node != obstacleStatics.end(); node++)
        {
            SceneObjectPtr so = boost::dynamic_pointer_cast<SceneObject>(*node);

            if (isColliding(so, EndEffectorActor::eStatic))
            {
                return true;
            }
        }

        return false;
    }

    bool EndEffectorActor::isColliding(EndEffectorPtr eef, EndEffectorPtr obstacle, EndEffector::ContactInfoVector& storeContacts)
    {
        std::vector<EndEffectorActorPtr> obstacleActors;
        obstacle->getActors(obstacleActors);

        std::vector<RobotNodePtr> obstacleStatics;
        obstacle->getStatics(obstacleStatics);

        for (std::vector<EndEffectorActorPtr>::iterator actor = obstacleActors.begin(); actor != obstacleActors.end(); actor++)
        {
            // Don't check for collisions with the actor itself
            if (((*actor)->getName() != name) && isColliding(eef, *actor, storeContacts))
            {
                return true;
            }
        }

        for (std::vector<RobotNodePtr>::iterator node = obstacleStatics.begin(); node != obstacleStatics.end(); node++)
        {
            SceneObjectPtr so = boost::dynamic_pointer_cast<SceneObject>(*node);

            if (isColliding(eef, so, storeContacts, EndEffectorActor::eStatic))
            {
                return true;
            }
        }

        return false;
    }

    bool EndEffectorActor::isColliding(EndEffectorPtr eef, EndEffectorActorPtr obstacle, EndEffector::ContactInfoVector& storeContacts)
    {
        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            SceneObjectPtr so = boost::dynamic_pointer_cast<SceneObject>(n->robotNode);

            if ((n->colMode & eActors) && obstacle->isColliding(eef, so, storeContacts))
            {
                return true;
            }
        }

        return false;

    }

    bool EndEffectorActor::isColliding(EndEffectorPtr eef, SceneObjectPtr obstacle, EndEffector::ContactInfoVector& storeContacts, CollisionMode checkColMode /*= EndEffectorActor::eAll*/)
    {
        if (!obstacle || !obstacle->getCollisionModel())
        {
            return false;
        }

        //Eigen::Vector3f contact;
        bool col = false;

        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {

            if ((n->colMode & checkColMode) &&
                n->robotNode->getCollisionModel() &&
                colChecker->checkCollision(n->robotNode->getCollisionModel(), obstacle->getCollisionModel()))
            {
                col = true;
                // create contact info
                EndEffector::ContactInfo ci;
                ci.eef = eef;
                ci.actor = shared_from_this();
                ci.robotNode = n->robotNode;
                ci.obstacle = obstacle;

                // todo: not needed here, later we calculate the distance with opened actors...
                int id1, id2;
                ci.distance = colChecker->calculateDistance(ci.robotNode->getCollisionModel(), ci.obstacle->getCollisionModel(), ci.contactPointFingerGlobal, ci.contactPointObstacleGlobal, &id1, &id2);
                ci.contactPointFingerLocal = ci.obstacle->toLocalCoordinateSystemVec(ci.contactPointFingerGlobal);
                ci.contactPointObstacleLocal = ci.obstacle->toLocalCoordinateSystemVec(ci.contactPointObstacleGlobal);


                storeContacts.push_back(ci);
            }
        }

        return col;
    }





    std::vector< RobotNodePtr > EndEffectorActor::getRobotNodes()
    {
        std::vector< RobotNodePtr > res;

        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            res.push_back(n->robotNode);
        }

        return res;
    }

    void EndEffectorActor::print()
    {
        cout << " ****" << endl;
        cout << " ** Name:" << name << endl;

        for (size_t i = 0; i < actors.size(); i++)
        {
            cout << " *** RobotNode: " << actors[i].robotNode->getName() << ", Direction/Speed:" << actors[i].directionAndSpeed << endl;
            //actors[i].robotNode->print();
        }

        cout << " ****" << endl;
    }

    bool EndEffectorActor::hasNode(RobotNodePtr node)
    {
        std::vector<ActorDefinition>::iterator iS = actors.begin();

        while (iS != actors.end())
        {
            if (iS->robotNode == node)
            {
                return true;
            }

            iS++;
        }

        return false;
    }

    bool EndEffectorActor::nodesSufficient(std::vector<RobotNodePtr> nodes) const
    {
        std::vector<ActorDefinition>::const_iterator i = actors.begin();

        while (i != actors.end())
        {
            std::vector<RobotNodePtr>::const_iterator j = nodes.begin();
            bool ok = false;

            while (j != nodes.end())
            {
                if (i->robotNode->getName() == (*j)->getName())
                {
                    ok = true;
                    break;
                }

                j++;
            }

            if (!ok)
            {
                return false;
            }

            i++;
        }

        return true;
    }

    float EndEffectorActor::getApproximatedLength()
    {
        BoundingBox bb_all;

        for (size_t j = 0; j < actors.size(); j++)
        {
            if (actors[j].robotNode->getCollisionModel())
            {
                BoundingBox bb = actors[j].robotNode->getCollisionModel()->getBoundingBox();
                bb_all.addPoint(bb.getMin());
                bb_all.addPoint(bb.getMax());
            }
        }

        Eigen::Vector3f d = bb_all.getMax() - bb_all.getMin();
        return d.norm();
    }

    VirtualRobot::RobotConfigPtr EndEffectorActor::getConfiguration()
    {
        if (actors.size() == 0 || !actors[0].robotNode)
        {
            return VirtualRobot::RobotConfigPtr();
        }

        std::vector< RobotConfig::Configuration > c;

        for (size_t i = 0; i < actors.size(); i++)
        {
            RobotConfig::Configuration e;
            e.name = actors[i].robotNode->getName();
            e.value = actors[i].robotNode->getJointValue();
            c.push_back(e);
        }

        RobotConfigPtr res(new RobotConfig(actors[0].robotNode->getRobot(), name, c));
        return res;
    }

    std::string EndEffectorActor::toXML(int ident /*= 1*/)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < ident; i++)
        {
            pre += t;
        }

        std::string tt = pre + t;
        std::string ttt = tt + t;
        ss << pre << "<Actor name='" << name << "'>" << endl;

        for (size_t i = 0; i < actors.size(); i++)
        {
            ss << tt << "<Node name='" << actors[i].robotNode->getName() << "' ";

            if (actors[i].colMode == eNone)
            {
                ss << "ConsiderCollisions='None' ";
            }

            if (actors[i].colMode == eAll)
            {
                ss << "ConsiderCollisions='All' ";
            }
            else
            {
                if (actors[i].colMode & eActors)
                {
                    ss << "ConsiderCollisions='Actors' ";
                }

                if (actors[i].colMode & eStatic)
                {
                    ss << "ConsiderCollisions='Static' ";
                }
            }

            ss << "Direction='" << actors[i].directionAndSpeed << "'/>" << endl;
        }

        ss << pre << "</Actor>" << endl;
        return ss.str();
    }



} // namespace VirtualRobot
