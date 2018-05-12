
#include "RobotNodeSet.h"
#include "SceneObjectSet.h"
#include "Robot.h"
#include "RobotConfig.h"
#include "VirtualRobotException.h"
#include "CollisionDetection/CollisionChecker.h"

#include <algorithm>

namespace VirtualRobot
{

    RobotNodeSet::RobotNodeSet(const std::string& name,
                               RobotWeakPtr robot,
                               const std::vector< RobotNodePtr >& robotNodes,
                               const RobotNodePtr kinematicRoot /*= RobotNodePtr()*/,
                               const RobotNodePtr tcp /*= RobotNodePtr()*/)
        : SceneObjectSet(name, robotNodes.size() > 0 ? robotNodes[0]->getCollisionChecker() : CollisionCheckerPtr())
    {
        this->robotNodes = robotNodes;
        this->kinematicRoot = kinematicRoot;
        this->tcp = tcp;
        RobotPtr rob = robot.lock();

        if (!kinematicRoot && robotNodes.size() > 0)
        {
            this->kinematicRoot = rob->getRootNode();
        }
        if (!isKinematicRoot(this->kinematicRoot))
        {
            VR_WARNING << "RobotNodeSet initialized with invalid kinematic root: Falling back to robot root node" << endl;
            this->kinematicRoot = rob->getRootNode();
        }

        if (!tcp && robotNodes.size() > 0)
        {
            this->tcp = robotNodes[robotNodes.size() - 1];
        }

        this->robot = robot;

        // now, the objects are stored in the parent's (SceneObjectSet) data structure, so that the methods of SceneObjectSet do work
        for (size_t i = 0; i < robotNodes.size(); i++)
        {
            SceneObjectPtr cm = boost::dynamic_pointer_cast<SceneObject>(robotNodes[i]);

            if (cm)
            {
                if (colChecker != cm->getCollisionChecker())
                {
                    VR_WARNING << "col model of " << robotNodes[i]->getName() << " belongs to different instance of collision checker, in: " << getName().c_str() << endl;
                }
                else
                {
                    sceneObjects.push_back(cm);
                }
            }
        }
    }


    RobotNodeSetPtr RobotNodeSet::createRobotNodeSet(RobotPtr robot,
            const std::string& name,
            const std::vector< std::string >& robotNodeNames,
            const std::string& kinematicRootName,
            const std::string& tcpName,
            bool registerToRobot)
    {
        VR_ASSERT(robot != NULL);
        std::vector< RobotNodePtr > robotNodes;

        if (robotNodeNames.empty())
        {
            VR_WARNING << " No robot nodes in set..." << endl;
        }
        else
        {
            for (unsigned int i = 0; i < robotNodeNames.size(); i++)
            {
                RobotNodePtr node = robot->getRobotNode(robotNodeNames[i]);
                THROW_VR_EXCEPTION_IF(!node, "No robot node with name " << robotNodeNames[i] << " found...");
                robotNodes.push_back(node);
            }
        }

        RobotNodePtr kinematicRoot;

        if (!kinematicRootName.empty())
        {
            RobotNodePtr node = robot->getRobotNode(kinematicRootName);
            THROW_VR_EXCEPTION_IF(!node, "No root node with name " << kinematicRootName << " found...");
            kinematicRoot = node;
        }
        else
        {
            if (!robotNodes.empty())
            {
                kinematicRoot = robotNodes[0];
            }
        }

        RobotNodePtr tcp;

        if (!tcpName.empty())
        {
            RobotNodePtr node = robot->getRobotNode(tcpName);
            THROW_VR_EXCEPTION_IF(!node, "No tcp node with name " << tcpName << " found...");
            tcp = node;
        }
        else
        {
            if (!robotNodes.empty())
            {
                tcp = robotNodes[robotNodes.size() - 1];
            }
        }

        RobotNodeSetPtr rns = RobotNodeSet::createRobotNodeSet(robot, name, robotNodes, kinematicRoot, tcp, registerToRobot);
        return rns;
    }


    RobotNodeSetPtr RobotNodeSet::createRobotNodeSet(RobotPtr robot,
            const std::string& name,
            const std::vector< RobotNodePtr >& robotNodes,
            const RobotNodePtr kinematicRoot,
            const RobotNodePtr tcp,
            bool registerToRobot)
    {
        VR_ASSERT(robot != NULL);

        if (robotNodes.empty() || !robotNodes[0])
        {
            VR_WARNING << " No robot nodes in set..." << endl;
        }
        else
        {
            CollisionCheckerPtr cc = robotNodes[0]->getCollisionChecker();

            for (unsigned int i = 1; i < robotNodes.size(); i++)
            {
                if (robotNodes[0]->getRobot() != robotNodes[i]->getRobot())
                {
                    THROW_VR_EXCEPTION("Robot nodes belong to different robots");
                }

                if (cc !=  robotNodes[i]->getCollisionChecker())
                {
                    THROW_VR_EXCEPTION("Robot nodes belong to different collision checkers");
                }
            }
        }

        RobotNodePtr kinematicRootNode = kinematicRoot;

        if (!kinematicRootNode)
        {
            kinematicRootNode = robot->getRootNode();
        }

        RobotNodePtr tcpNode = tcp;

        if (!tcpNode)
        {
            THROW_VR_EXCEPTION_IF(robotNodes.empty(), "can not determine the tcp node need for creating a RobotNodeSet");
            tcpNode = robotNodes[robotNodes.size() - 1];
        }

        RobotNodeSetPtr rns(new RobotNodeSet(name, robot, robotNodes, kinematicRootNode, tcpNode));

        if (registerToRobot)
        {
            THROW_VR_EXCEPTION_IF(robot->hasRobotNodeSet(rns), "RobotNodeSet with name " << rns->getName() << " already present in the robot");
            robot->registerRobotNodeSet(rns);
        }

        return rns;
    }

    RobotNodePtr RobotNodeSet::getTCP() const
    {
        return tcp;
    }

    RobotNodeSetPtr RobotNodeSet::clone(RobotPtr newRobot, const RobotNodePtr newKinematicRoot)
    {
        if (!newRobot)
        {
            VR_ERROR << "Attempting to clone RobotNodeSet for invalid robot";
            return RobotNodeSetPtr();
        }

        std::vector<std::string> nodeNames(robotNodes.size());

        for (unsigned int i = 0; i < robotNodes.size(); i++)
        {
            nodeNames[i] = robotNodes[i]->getName();
        }

        std::string kinRootName = "";

        if (kinematicRoot)
        {
            kinRootName = kinematicRoot->getName();
        }
        if (newKinematicRoot)
        {
            kinRootName = newKinematicRoot->getName();
        }

        std::string tcpName = "";

        if (tcp)
        {
            tcpName = tcp->getName();
        }

        RobotNodeSetPtr nodeSet = RobotNodeSet::createRobotNodeSet(newRobot, name, nodeNames, kinRootName, tcpName, true);
        return nodeSet;
    }


    RobotPtr RobotNodeSet::getRobot()
    {
        RobotPtr rob = robot.lock();
        return rob;
    }

    bool RobotNodeSet::hasRobotNode(RobotNodePtr robotNode) const
    {
        for (unsigned int i = 0; i < robotNodes.size(); i++)
        {
            if (robotNodes[i] == robotNode)
            {
                return true;
            }
        }

        return false;
    }


    const std::vector< RobotNodePtr > RobotNodeSet::getAllRobotNodes() const
    {
        return robotNodes;
    }

    RobotNodePtr RobotNodeSet::getKinematicRoot() const
    {
        return kinematicRoot;
    }

    void RobotNodeSet::setKinematicRoot(RobotNodePtr robotNode)
    {
        kinematicRoot = robotNode;
    }

    unsigned int RobotNodeSet::getSize() const
    {
        return robotNodes.size();
    }

    void RobotNodeSet::print() const
    {
        cout << "  Robot Node Set <" << name << ">" << endl;
        cout << "  Root node: ";

        if (kinematicRoot)
        {
            cout << kinematicRoot->getName() << endl;
        }
        else
        {
            cout << "<not set>" << endl;
        }

        cout << "  TCP node: ";

        if (tcp)
        {
            cout << tcp->getName() << endl;
        }
        else
        {
            cout << "<not set>" << endl;
        }

        for (unsigned int i = 0; i < robotNodes.size(); i++)
        {
            cout << "  Node " << i << ": ";

            if (robotNodes[i])
            {
                cout << robotNodes[i]->getName() << endl;
            }
            else
            {
                cout << "<not set>" << endl;
            }
        }
    }

    void RobotNodeSet::getJointValues(std::vector<float>& fillVector) const
    {
        fillVector.resize(robotNodes.size());
        std::transform(robotNodes.begin(), robotNodes.end(), fillVector.begin(), boost::mem_fn(&RobotNode::getJointValue));
    }

    void RobotNodeSet::getJointValues(Eigen::VectorXf& fillVector) const
    {
        fillVector.resize(robotNodes.size());

        for (size_t i = 0; i < robotNodes.size(); i++)
        {
            fillVector[i] = robotNodes[i]->getJointValue();
        }

    }

    void RobotNodeSet::getJointValues(RobotConfigPtr fillVector) const
    {
        THROW_VR_EXCEPTION_IF(!fillVector, "NULL data");

        for (size_t i = 0; i < robotNodes.size(); i++)
        {
            fillVector->setConfig(robotNodes[i]->getName(), robotNodes[i]->getJointValue());
        }
    }

    std::vector<float> RobotNodeSet::getJointValues() const
    {
        std::vector<float> res;
        getJointValues(res);
        return res;
    }


    void RobotNodeSet::respectJointLimits(std::vector<float>& jointValues) const
    {
        THROW_VR_EXCEPTION_IF(jointValues.size() != robotNodes.size(), "Wrong vector dimension (robotNodes:" << robotNodes.size() << ", jointValues: " << jointValues.size() << ")" << endl);

        for (unsigned int i = 0; i < robotNodes.size(); i++)
        {
            robotNodes[i]->respectJointLimits(jointValues[i]);
        }
    }

    void RobotNodeSet::respectJointLimits(Eigen::VectorXf& jointValues) const
    {
        for (unsigned int i = 0; i < robotNodes.size(); i++)
        {
            robotNodes[i]->respectJointLimits(jointValues[i]);
        }
    }

    void RobotNodeSet::setJointValues(const std::vector<float>& jointValues)
    {
        THROW_VR_EXCEPTION_IF(jointValues.size() != robotNodes.size(), "Wrong vector dimension (robotNodes:" << robotNodes.size() << ", jointValues: " << jointValues.size() << ")" << endl);

        RobotPtr rob = robot.lock();
        VR_ASSERT(rob);
        WriteLockPtr lock = rob->getWriteLock();

        for (unsigned int i = 0; i < robotNodes.size(); i++)
        {
            robotNodes[i]->setJointValueNoUpdate(jointValues[i]);
        }

        if (kinematicRoot)
        {
            kinematicRoot->updatePose();
        }
        else
        {
            rob->applyJointValues();
        }
    }


    void RobotNodeSet::setJointValues(const Eigen::VectorXf& jointValues)
    {
        THROW_VR_EXCEPTION_IF(jointValues.rows() != robotNodes.size(), "Wrong vector dimension (robotNodes:" << robotNodes.size() << ", jointValues: " << jointValues.size() << ")" << endl);
        RobotPtr rob = robot.lock();
        VR_ASSERT(rob);
        WriteLockPtr lock = rob->getWriteLock();

        for (unsigned int i = 0; i < robotNodes.size(); i++)
        {
            robotNodes[i]->setJointValueNoUpdate(jointValues[i]);
        }

        if (kinematicRoot)
        {
            kinematicRoot->updatePose();
        }
        else
        {
            rob->applyJointValues();
        }
    }

    void RobotNodeSet::setJointValues(const RobotConfigPtr jointValues)
    {
        VR_ASSERT(jointValues);
        RobotPtr rob = robot.lock();
        VR_ASSERT(rob);
        WriteLockPtr lock = rob->getWriteLock();

        for (unsigned int i = 0; i < robotNodes.size(); i++)
        {
            if (jointValues->hasConfig(robotNodes[i]->getName()))
            {
                robotNodes[i]->setJointValueNoUpdate(jointValues->getConfig(robotNodes[i]->getName()));
            }
        }

        if (kinematicRoot)
        {
            kinematicRoot->updatePose();
        }
        else
        {
            rob->applyJointValues();
        }

    }



    RobotNodePtr& RobotNodeSet::getNode(int i)
    {
        THROW_VR_EXCEPTION_IF((i >= (int)robotNodes.size() || i < 0), "Index out of bounds:" << i << ", (should be between 0 and " << (robotNodes.size() - 1));

        return robotNodes[i];
    }

    RobotNodePtr& RobotNodeSet::operator[](int i)
    {
        THROW_VR_EXCEPTION_IF((i >= (int)robotNodes.size() || i < 0), "Index out of bounds:" << i << ", (should be between 0 and " << (robotNodes.size() - 1));

        return robotNodes[i];
    }


    bool RobotNodeSet::isKinematicChain()
    {
        for (unsigned int i = 0; i < this->robotNodes.size() - 1; i++)
        {
            if (!this->robotNodes[i]->hasChild(this->robotNodes[i + 1], true))
            {
                return false;
            }
        }

        return true;
    }


    void RobotNodeSet::highlight(VisualizationPtr visualization, bool enable)
    {
        if (!visualization)
        {
            return;
        }

        for (unsigned int i = 0; i < this->robotNodes.size(); i++)
        {
            robotNodes[i]->highlight(visualization, enable);
        }
    }

    int RobotNodeSet::getNumFaces(bool collisionModel)
    {
        int res = 0;

        for (unsigned int i = 0; i < this->robotNodes.size(); i++)
        {
            res += robotNodes[i]->getNumFaces(collisionModel);
        }

        return res;
    }
    /*
    VirtualRobot::SceneObjectSetPtr RobotNodeSet::createSceneObjectSet()
    {
        CollisionCheckerPtr cc = getCollisionChecker();

        SceneObjectSetPtr cms(new SceneObjectSet(name,cc));
        cms->addSceneObjects(shared_from_this());
        return cms;
    }*/

    float RobotNodeSet::getMaximumExtension()
    {
        float result = 0;
        Eigen::Matrix4f t;
        Eigen::Vector3f v;

        if (kinematicRoot && robotNodes.size() > 0)
        {
            t = kinematicRoot->getTransformationTo(robotNodes[0]);
            v = MathTools::getTranslation(t);
            result += v.norm();
        }

        for (size_t i = 0; i < this->robotNodes.size() - 1; i++)
        {
            t = robotNodes[i]->getTransformationTo(robotNodes[i + 1]);
            v = MathTools::getTranslation(t);
            result += v.norm();
        }

        if (tcp && robotNodes.size() > 0)
        {
            t = tcp->getTransformationTo(robotNodes[robotNodes.size() - 1]);
            v = MathTools::getTranslation(t);
            result += v.norm();
        }

        return result;
    }

    Eigen::Vector3f RobotNodeSet::getCoM()
    {
        Eigen::Vector3f res;
        res.setZero();

        float m = getMass();

        if (m <= 0)
        {
            return res;
        }

        for (size_t i = 0; i < this->robotNodes.size(); i++)
        {
            res += robotNodes[i]->getCoMGlobal() * robotNodes[i]->getMass() / m;
        }

        return res;
    }

    float RobotNodeSet::getMass()
    {
        float res = 0;

        for (size_t i = 0; i < this->robotNodes.size(); i++)
        {
            res += robotNodes[i]->getMass();
        }

        return res;
    }

    bool RobotNodeSet::nodesSufficient(std::vector<RobotNodePtr> nodes) const
    {
        bool tcpOk = false;
        bool krOk = false;

        if (!tcp)
        {
            tcpOk = true;
        }

        if (!kinematicRoot)
        {
            krOk = true;
        }

        std::vector<RobotNodePtr>::const_iterator j = nodes.begin();
        bool ok = false;

        while (j != nodes.end())
        {

            if (!tcpOk && (*j)->getName() == tcp->getName())
            {
                tcpOk = true;
            }

            if (!krOk && (*j)->getName() == kinematicRoot->getName())
            {
                krOk = true;
            }

            j++;
        }

        if (!krOk || !tcpOk)
        {
            return false;
        }

        std::vector<RobotNodePtr>::const_iterator i = robotNodes.begin();

        while (i != robotNodes.end())
        {
            std::vector<RobotNodePtr>::const_iterator j = nodes.begin();
            bool ok = false;

            while (j != nodes.end())
            {
                if ((*i)->getName() == (*j)->getName())
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

    bool RobotNodeSet::checkJointLimits(std::vector<float>& jointValues, bool verbose /*= false*/) const
    {
        THROW_VR_EXCEPTION_IF(jointValues.size() != robotNodes.size(), "Wrong vector dimension (robotNodes:" << robotNodes.size() << ", jointValues: " << jointValues.size() << ")" << endl);

        bool res = true;

        for (unsigned int i = 0; i < robotNodes.size(); i++)
        {
            res = res & robotNodes[i]->checkJointLimits(jointValues[i], verbose);
        }

        return res;
    }

    bool RobotNodeSet::checkJointLimits(Eigen::VectorXf& jointValues, bool verbose /*= false*/) const
    {
        THROW_VR_EXCEPTION_IF(jointValues.size() != robotNodes.size(), "Wrong vector dimension (robotNodes:" << robotNodes.size() << ", jointValues: " << jointValues.size() << ")" << endl);

        bool res = true;

        for (unsigned int i = 0; i < robotNodes.size(); i++)
        {
            res = res & robotNodes[i]->checkJointLimits(jointValues[i], verbose);
        }

        if (!res && verbose)
        {
            VR_INFO << "RobotNodeSet: " << getName() << ": joint limits are violated" << endl;
        }

        return res;
    }

    bool RobotNodeSet::addSceneObject(SceneObjectPtr sceneObject)
    {
        THROW_VR_EXCEPTION("Not allowed for RobotNodeSets.");
        return false;
    }

    bool RobotNodeSet::addSceneObjects(SceneObjectSetPtr sceneObjectSet)
    {
        THROW_VR_EXCEPTION("Not allowed for RobotNodeSets.");
        return false;
    }

    bool RobotNodeSet::addSceneObjects(RobotNodeSetPtr robotNodeSet)
    {
        THROW_VR_EXCEPTION("Not allowed for RobotNodeSets.");
        return false;
    }

    bool RobotNodeSet::addSceneObjects(std::vector<RobotNodePtr> robotNodes)
    {
        THROW_VR_EXCEPTION("Not allowed for RobotNodeSets.");
        return false;
    }

    bool RobotNodeSet::removeSceneObject(SceneObjectPtr sceneObject)
    {
        THROW_VR_EXCEPTION("Not allowed for RobotNodeSets.");
        return false;
    }

    bool RobotNodeSet::isKinematicRoot(RobotNodePtr robotNode)
    {
        RobotNodePtr node;
        for(size_t i=0; i<robotNodes.size(); i++)
        {
            node = robotNodes.at(i);
            if(node != robotNode && !robotNode->hasChild(node, true))
            {
                return false;
            }
        }

        return true;
    }



    std::string RobotNodeSet::toXML(int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += "\t";
        }

        ss << pre << "<RobotNodeSet name='" << name << "'>\n";

        for (size_t i = 0; i < robotNodes.size(); i++)
        {
            ss << pre << t << "<Node name='" << robotNodes[i]->getName() << "'/>\n";
        }

        ss << pre << "</RobotNodeSet>\n";
        return ss.str();
    }



} // namespace VirtualRobot
