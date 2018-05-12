
#include "Robot.h"
#include "RobotConfig.h"
#include "Trajectory.h"
#include "VirtualRobotException.h"
#include "CollisionDetection/CollisionChecker.h"
#include "EndEffector/EndEffector.h"


namespace VirtualRobot
{

    Robot::Robot(const std::string& name, const std::string& type)
        : SceneObject(name)
    {
        this->type = type;
        updateVisualization = true;
        use_mutex = true;
    }

    Robot::Robot()
    {
    }

    Robot::~Robot()
    {
    }

    LocalRobot::~LocalRobot()
    {
        // clean up connections
        std::map< std::string, RobotNodePtr > ::iterator it = robotNodeMap.begin();

        while (it != robotNodeMap.end())
        {
            std::vector<SceneObjectPtr> c = it->second->getChildren();

            for (size_t j = 0; j < c.size(); j++)
            {
                it->second->detachChild(c[j]);
            }

            it++;
        }

        int i = (int)rootNode.use_count();
        robotNodeSetMap.clear();
        robotNodeMap.clear();
        i = (int)rootNode.use_count();
        rootNode.reset();
        //i = (int)rootNode.use_count();
    }

    LocalRobot::LocalRobot(const std::string& name, const std::string& type) : Robot(name, type)
    {
    }

    void LocalRobot::setRootNode(RobotNodePtr node)
    {
        attachChild(node);
        rootNode = node;
        node->initialize();

        //robotNodeMap.clear();
        if (!node)
        {
            VR_WARNING << "NULL root node..." << endl;
        }
        else
        {
            // create all globalposes
            applyJointValues();

            std::vector< RobotNodePtr > allNodes;
            node->collectAllRobotNodes(allNodes);

            for (unsigned int i = 0; i < allNodes.size(); i++)
            {
                std::string name = allNodes[i]->getName();

                if (!this->hasRobotNode(name))
                {
                    VR_WARNING << "Robot node with name <" << name << "> was not registered, adding it to RobotNodeMap" << endl;
                    registerRobotNode(allNodes[i]);
                }
            }
        }
    }

    void Robot::setThreadsafe(bool flag)
    {
        use_mutex = flag;
    }

    RobotNodePtr LocalRobot::getRobotNode(const std::string& robotNodeName)
    {
        if (robotNodeMap.find(robotNodeName) == robotNodeMap.end())
        {
            VR_WARNING << "No robot node with name <" << robotNodeName << "> defined." << endl;
            return RobotNodePtr();
        }

        return robotNodeMap[robotNodeName];
    }

    void LocalRobot::registerRobotNode(RobotNodePtr node)
    {
        if (!node)
        {
            return;
        }

        if (robotNodeMap.size() > 0)
        {
            // check for collision checker
            if (node->getCollisionChecker() != robotNodeMap.begin()->second->getCollisionChecker())
            {
                THROW_VR_EXCEPTION("Different Collision Checkers in " << node->getName() << " and " << robotNodeMap.begin()->second->getName());
            }
        }

        std::string robotNodeName = node->getName();

        if (robotNodeMap.find(robotNodeName) != robotNodeMap.end())
        {
            THROW_VR_EXCEPTION("There are (at least) two robot nodes with name <" << robotNodeName << "> defined, the second one is skipped!");
        }
        else
        {
            robotNodeMap[robotNodeName] = node;
        }
    }

    bool LocalRobot::hasRobotNode(RobotNodePtr node)
    {
        if (!node)
        {
            return false;
        }

        std::string robotNodeName = node->getName();

        if (this->hasRobotNode(robotNodeName))
        {
            return (this->getRobotNode(robotNodeName) == node);
        }

        return false;
    }

    bool LocalRobot::hasRobotNode(const std::string& robotNodeName)
    {
        return (robotNodeMap.find(robotNodeName) != robotNodeMap.end());
    }


    void LocalRobot::deregisterRobotNode(RobotNodePtr node)
    {
        if (!node)
        {
            return;
        }

        std::string robotNodeName = node->getName();
        std::map< std::string, RobotNodePtr >::iterator i = robotNodeMap.find(robotNodeName);

        if (i != robotNodeMap.end())
        {
            robotNodeMap.erase(i);
        }
    }


    void LocalRobot::registerRobotNodeSet(RobotNodeSetPtr nodeSet)
    {
        if (!nodeSet)
        {
            return;
        }

        std::string nodeSetName = nodeSet->getName();

        if (robotNodeSetMap.find(nodeSetName) != robotNodeSetMap.end())
        {
            VR_WARNING << "There are (at least) two robot node sets with name <" << nodeSetName << "> defined, the second one overwrites first definition!" << endl;
            // overwrite
        }

        robotNodeSetMap[nodeSetName] = nodeSet;
    }

    bool Robot::hasRobotNodeSet(RobotNodeSetPtr nodeSet)
    {
        if (!nodeSet)
        {
            return false;
        }

        std::string nodeSetName = nodeSet->getName();

        return hasRobotNodeSet(nodeSetName);
    }

    bool LocalRobot::hasRobotNodeSet(const std::string& name)
    {
        if (robotNodeSetMap.find(name) != robotNodeSetMap.end())
        {
            return true;
        }

        return false;
    }

    void LocalRobot::deregisterRobotNodeSet(RobotNodeSetPtr nodeSet)
    {
        if (!nodeSet)
        {
            return;
        }

        std::string nodeSetName = nodeSet->getName();
        std::map< std::string, RobotNodeSetPtr >::iterator i = robotNodeSetMap.find(nodeSetName);

        if (i != robotNodeSetMap.end())
        {
            robotNodeSetMap.erase(i);
        }
    }


    /**
     * This method registers \p endEffector with the current VirtualRobot::Robot
     * instance.
     * It throws an exception if a VirtualRobot::EndEffector with the same name
     * has already been registered.
     */
    void LocalRobot::registerEndEffector(EndEffectorPtr endEffector)
    {
        if (!endEffector)
        {
            return;
        }

        std::string endEffectorName = endEffector->getName();

        if (endEffectorMap.find(endEffectorName) != endEffectorMap.end())
        {
            THROW_VR_EXCEPTION("Trying to register a second endeffector with name <" << endEffectorName << ">");
        }
        else
        {
            endEffectorMap[endEffectorName] = endEffector;
        }
    }


    /**
     * \return true if instance of VirtualRobot::Robot contains a reference to \p endEffector and false otherwise
     */
    bool Robot::hasEndEffector(EndEffectorPtr endEffector)
    {
        if (!endEffector)
        {
            return false;
        }

        std::string endEffectorName = endEffector->getName();

        if (this->hasEndEffector(endEffectorName))
        {
            return (this->getEndEffector(endEffectorName) == endEffector);
        }

        return false;
    }


    /**
     * \return true if instance of VirtualRobot::Robot contains an endeffector with name \p endEffectorName and false otherwise
     */
    bool LocalRobot::hasEndEffector(const std::string& endEffectorName)
    {
        if (endEffectorName.empty())
        {
            return false;
        }

        if (endEffectorMap.find(endEffectorName) == endEffectorMap.end())
        {
            return false;
        }

        return true;
    }


    /**
     * \return reference to endeffector with name \p endEffectorName or Null-Pointer otherwise
     */
    EndEffectorPtr LocalRobot::getEndEffector(const std::string& endEffectorName)
    {
        if (endEffectorMap.find(endEffectorName) == endEffectorMap.end())
        {
            VR_WARNING << "No endeffector node with name <" << endEffectorName << "> defined." << endl;
            return EndEffectorPtr();
        }

        return endEffectorMap[endEffectorName];
    }

    /**
     * This method stores all endeffectors belonging to the robot in \p storeEEF.
     * If there are no registered endeffectors \p storeEEF will be empty.
     */
    void LocalRobot::getEndEffectors(std::vector<EndEffectorPtr>& storeEEF)
    {
        storeEEF.clear();
        storeEEF.reserve(endEffectorMap.size());
        std::map<std::string, EndEffectorPtr>::const_iterator iterator = endEffectorMap.begin();

        while (endEffectorMap.end() != iterator)
        {
            storeEEF.push_back(iterator->second);
            ++iterator;
        }
    }

    std::vector<EndEffectorPtr> Robot::getEndEffectors()
    {
        std::vector<EndEffectorPtr> res;
        getEndEffectors(res);
        return res;
    }


    /**
     * \return VirtualRobot::Robot::name
     */
    std::string Robot::getName()
    {
        return name;
    }

    /**
     * \return VirtualRobot::Robot::type
     */
    std::string Robot::getType()
    {
        return type;
    }

    void Robot::print()
    {
        cout << "******** Robot ********" << endl;
        cout << "* Name: " << name << endl;
        cout << "* Type: " << type << endl;

        if (this->getRootNode())
        {
            cout << "* Root Node: " << this->getRootNode()->getName() << endl;
        }
        else
        {
            cout << "* Root Node: not set" << endl;
        }

        cout << endl;

        if (this->getRootNode())
        {
            this->getRootNode()->print(true, true);
        }

        cout << endl;

        std::vector<RobotNodeSetPtr> robotNodeSets = this->getRobotNodeSets();

        if (robotNodeSets.size() > 0)
        {
            cout << "* RobotNodeSets:" << endl;

            std::vector<RobotNodeSetPtr>::iterator iter = robotNodeSets.begin();

            while (iter != robotNodeSets.end())
            {
                cout << "----------------------------------" << endl;
                (*iter)->print();
                iter++;
            }

            cout << endl;
        }

        cout << "******** Robot ********" << endl;
    }


    /**
     * This method returns a reference to Robot::rootNode;
     */
    RobotNodePtr LocalRobot::getRootNode()
    {
        return this->rootNode;
    }

    /**
     * Update the transformations of all joints
     */
    void Robot::applyJointValues()
    {
        WriteLock(mutex, use_mutex);
        this->getRootNode()->updatePose(this->getGlobalPose());
    }

    /**
     * Can be called internally, when lock is already set.
     */
    void Robot::applyJointValuesNoLock()
    {
        this->getRootNode()->updatePose(this->getGlobalPose());
    }

    /**
     * This method stores all nodes belonging to the robot in \p storeNodes.
     * If there are no registered nodes \p storeNodes will be empty.
     */
    void LocalRobot::getRobotNodes(std::vector< RobotNodePtr >& storeNodes, bool clearVector /*=true*/)
    {
        if (clearVector)
        {
            storeNodes.clear();
        }

        storeNodes.reserve(robotNodeMap.size());
        std::map< std::string, RobotNodePtr>::const_iterator iterator = robotNodeMap.begin();

        while (robotNodeMap.end() != iterator)
        {
            storeNodes.push_back(iterator->second);
            ++iterator;
        }
    }

    std::vector< RobotNodePtr > Robot::getRobotNodes()
    {
        std::vector< RobotNodePtr > res;
        getRobotNodes(res);
        return res;
    }




    void Robot::setUpdateVisualization(bool enable)
    {
        updateVisualization = enable;

        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr> ::const_iterator iterator = robotNodes.begin();

        //  std::map< std::string, RobotNodePtr>::const_iterator iterator = robotNodeMap.begin();
        //  while(robotNodeMap.end() != iterator)
        while (robotNodes.end() != iterator)
        {
            //iterator->second->setUpdateVisualization(enable);
            (*iterator)->setUpdateVisualization(enable);
            ++iterator;
        }
    }

    bool Robot::getUpdateVisualizationStatus()
    {
        return updateVisualization;
    }

    RobotNodeSetPtr LocalRobot::getRobotNodeSet(const std::string& nodeSetName)
    {
        if (robotNodeSetMap.find(nodeSetName) == robotNodeSetMap.end())
        {
            VR_WARNING << "No robot node set with name <" << nodeSetName << "> defined." << endl;
            return RobotNodeSetPtr();
        }

        return robotNodeSetMap[nodeSetName];
    }

    /**
     * This method stores all endeffectors belonging to the robot in \p storeEEF.
     * If there are no registered endeffectors \p storeEEF will be empty.
     */
    void LocalRobot::getRobotNodeSets(std::vector<RobotNodeSetPtr>& storeNodeSets)
    {
        storeNodeSets.clear();
        storeNodeSets.reserve(robotNodeSetMap.size());
        std::map<std::string, RobotNodeSetPtr>::const_iterator iterator = robotNodeSetMap.begin();

        while (robotNodeSetMap.end() != iterator)
        {
            storeNodeSets.push_back(iterator->second);
            ++iterator;
        }
    }

    std::vector<RobotNodeSetPtr> Robot::getRobotNodeSets()
    {
        std::vector<RobotNodeSetPtr> res;
        getRobotNodeSets(res);
        return res;
    }


    void Robot::highlight(VisualizationPtr visualization, bool enable)
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            (*iterator)->highlight(visualization, enable);
            ++iterator;
        }
    }

    void Robot::showStructure(bool enable, const std::string& type)
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            (*iterator)->showStructure(enable, type);
            ++iterator;
        }

    }

    void Robot::showPhysicsInformation(bool enableCoM, bool enableInertial, VisualizationNodePtr comModel)
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            (*iterator)->showPhysicsInformation(enableCoM, enableInertial, comModel);
            ++iterator;
        }

    }

    void Robot::showCoordinateSystems(bool enable, const std::string& type)
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            (*iterator)->showCoordinateSystem(enable, 1.0f, NULL, type);
            ++iterator;
        }
    }

    void Robot::setupVisualization(bool showVisualization, bool showAttachedVisualizations)
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            (*iterator)->setupVisualization(showVisualization, showAttachedVisualizations);
            ++iterator;
        }

    }

    int Robot::getNumFaces(bool collisionModel)
    {
        int res = 0;
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            res += (*iterator)->getNumFaces(collisionModel);
            ++iterator;
        }

        return res;
    }

    VirtualRobot::CollisionCheckerPtr Robot::getCollisionChecker()
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();

        if (robotNodes.size() == 0)
        {
            return CollisionChecker::getGlobalCollisionChecker();
        }

        return (*robotNodes.begin())->getCollisionChecker();
    }

    void LocalRobot::setGlobalPose(const Eigen::Matrix4f& globalPose)
    {
        setGlobalPose(globalPose, true);
    }

    void LocalRobot::setGlobalPose(const Eigen::Matrix4f& globalPose, bool applyJointValues /*= true*/)
    {
        WriteLock(mutex, use_mutex);
        this->globalPose = globalPose;

        if (applyJointValues)
        {
            applyJointValuesNoLock();
        }
    }

    Eigen::Matrix4f LocalRobot::getGlobalPose()
    {
        ReadLock(mutex, use_mutex);
        return globalPose;
    }


    Eigen::Vector3f Robot::getCoMLocal()
    {
        Eigen::Vector3f com = getCoMGlobal();
        return toLocalCoordinateSystemVec(com);
    }

    Eigen::Vector3f Robot::getCoMGlobal()
    {
        Eigen::Vector3f res;
        res.setZero();

        float m = getMass();

        if (m <= 0)
        {
            return res;
        }

        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            res += (*iterator)->getCoMGlobal() * (*iterator)->getMass() / m;

            iterator++;
        }

        return res;
    }

    float Robot::getMass()
    {
        float res = 0;
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            res += (*iterator)->getMass();
            iterator++;
        }

        return res;
    }

    std::vector< CollisionModelPtr > Robot::getCollisionModels()
    {
        std::vector< CollisionModelPtr > result;
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            if ((*iterator)->getCollisionModel())
            {
                result.push_back((*iterator)->getCollisionModel());
            }

            iterator++;
        }

        return result;
    }

    VirtualRobot::RobotPtr Robot::extractSubPart(RobotNodePtr startJoint, const std::string& newRobotType, const std::string& newRobotName, bool cloneRNS, bool cloneEEFs, CollisionCheckerPtr collisionChecker, float scaling)
    {
        THROW_VR_EXCEPTION_IF(!hasRobotNode(startJoint), " StartJoint is not part of this robot");
        THROW_VR_EXCEPTION_IF(scaling <= 0, " Scaling must be >0.");

        CollisionCheckerPtr colChecker = collisionChecker;

        if (!colChecker)
        {
            colChecker = this->getCollisionChecker();
        }

        //stefan Warning!!!!! which robot-type to create
        RobotPtr result(new LocalRobot(newRobotName, newRobotType));

        RobotNodePtr rootNew = startJoint->clone(result, true, RobotNodePtr(), colChecker, scaling);
        THROW_VR_EXCEPTION_IF(!rootNew, "Clone failed...");
        result->setRootNode(rootNew);

        std::vector<RobotNodePtr> rn = result->getRobotNodes();

        // check for RNS that are covered by subpart
        if (cloneRNS)
        {
            std::vector<RobotNodeSetPtr> robotNodeSets = this->getRobotNodeSets();
            std::vector<RobotNodeSetPtr>::const_iterator iterator = robotNodeSets.begin();

            while (robotNodeSets.end() != iterator)
            {
                if ((*iterator)->nodesSufficient(rn))
                {
                    RobotNodeSetPtr rns = (*iterator)->clone(result);

                    // already done in rns->clone()
                    //if (rns)
                    //  result->registerRobotNodeSet(rns);
                }

                ++iterator;
            }
        }

        if (cloneEEFs)
        {
            std::vector<EndEffectorPtr> endEffectors = this->getEndEffectors();
            std::vector<EndEffectorPtr>::const_iterator iterator = endEffectors.begin();

            while (endEffectors.end() != iterator)
            {
                if ((*iterator)->nodesSufficient(rn))
                {
                    // registers eef to result:
                    EndEffectorPtr eef = (*iterator)->clone(result);
                }

                ++iterator;
            }
        }

        std::vector<RobotNodePtr> allNodes;
        startJoint->collectAllRobotNodes(allNodes);

        for (size_t i = 0; i < allNodes.size(); i++)
        {
            RobotNodePtr roN = result->getRobotNode(allNodes[i]->getName());

            if (roN)
            {
                roN->setJointValueNoUpdate(allNodes[i]->getJointValue());
            }
        }

        result->applyJointValues();
        return result;
    }

    void Robot::setGlobalPoseForRobotNode(const RobotNodePtr& node, const Eigen::Matrix4f& globalPoseNode)
    {
        THROW_VR_EXCEPTION_IF(!node, "NULL node");

        // get transformation from current to wanted tcp pose
        Eigen::Matrix4f t = globalPoseNode * node->getGlobalPose().inverse();

        // apply transformation to current global pose of robot
        t = t * getGlobalPose();

        // set t
        setGlobalPose(t);
    }

    VirtualRobot::RobotPtr Robot::clone(const std::string& name, CollisionCheckerPtr collisionChecker, float scaling)
    {
        VirtualRobot::RobotPtr result = extractSubPart(this->getRootNode(), this->getType(), name, true, true, collisionChecker, scaling);
        result->setGlobalPose(getGlobalPose());
        return result;
    }

    void Robot::createVisualizationFromCollisionModels()
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            RobotNodePtr rn = *iterator;

            if (rn->getVisualization(SceneObject::Full) && rn->getVisualization(SceneObject::Collision))
            {
                VisualizationNodePtr v = rn->getVisualization(SceneObject::Collision)->clone();
                rn->setVisualization(v);
            }

            iterator++;
        }
    }

    VirtualRobot::RobotConfigPtr Robot::getConfig()
    {
        RobotConfigPtr r(new RobotConfig(shared_from_this(), getName()));

        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            RobotNodePtr rn = *iterator;

            if (rn->isTranslationalJoint() || rn->isRotationalJoint())
            {
                r->setConfig(rn, rn->getJointValue());
            }

            iterator++;
        }

        return r;
    }

    bool Robot::setConfig(RobotConfigPtr c)
    {
        if (!c)
        {
            return false;
        }

        setJointValues(c);
        return true;
    }

    void Robot::setFilename(const std::string& filename)
    {
        this->filename = filename;
    }

    std::string Robot::getFilename()
    {
        return filename;
    }

    ReadLockPtr Robot::getReadLock()
    {
        return ReadLockPtr(new ReadLock(mutex, use_mutex));
    }

    WriteLockPtr Robot::getWriteLock()
    {
        return WriteLockPtr(new WriteLock(mutex, use_mutex));
    }

    void Robot::setJointValue(const std::string& nodeName, float jointValue)
    {
        RobotNodePtr rn = getRobotNode(nodeName);
        setJointValue(rn, jointValue);
    }

    void Robot::setJointValue(RobotNodePtr rn, float jointValue)
    {
        VR_ASSERT(rn);
        rn->setJointValue(jointValue);
    }

    void Robot::setJointValues(const std::map< std::string, float >& jointValues)
    {
        WriteLock(mutex, use_mutex);

        std::map< std::string, float >::const_iterator it = jointValues.begin();

        while (it != jointValues.end())
        {
            RobotNodePtr rn = getRobotNode(it->first);
            VR_ASSERT(rn);
            rn->setJointValueNoUpdate(it->second);
            it++;
        }

        applyJointValuesNoLock();
    }

    void Robot::setJointValues(const std::map<RobotNodePtr, float>& jointValues)
    {
        WriteLock(mutex, use_mutex);

        std::map< RobotNodePtr, float >::const_iterator it = jointValues.begin();

        while (it != jointValues.end())
        {
            RobotNodePtr rn = it->first;
            rn->setJointValueNoUpdate(it->second);
            it++;
        }

        applyJointValuesNoLock();
    }

    void Robot::setJointValues(RobotNodeSetPtr rns, const std::vector<float>& jointValues)
    {
        VR_ASSERT(rns);
        rns->setJointValues(jointValues);
    }

    void Robot::setJointValues(RobotNodeSetPtr rns, const Eigen::VectorXf& jointValues)
    {
        VR_ASSERT(rns);
        rns->setJointValues(jointValues);
    }

    void Robot::setJointValues(RobotConfigPtr config)
    {
        VR_ASSERT(config);
        config->setJointValues(shared_from_this());
    }

    void Robot::setJointValues(TrajectoryPtr trajectory, float t)
    {
        VR_ASSERT(trajectory);
        Eigen::VectorXf c;
        trajectory->interpolate(t, c);
        setJointValues(trajectory->getRobotNodeSet(), c);
    }

    void Robot::setJointValues(RobotNodeSetPtr rns, RobotConfigPtr config)
    {
        VR_ASSERT(rns);
        VR_ASSERT(config);
        rns->setJointValues(config);
    }

    void Robot::setJointValues(const std::vector<RobotNodePtr> rn, const std::vector<float>& jointValues)
    {
        VR_ASSERT(rn.size() == jointValues.size());
        WriteLock(mutex, use_mutex);

        for (size_t i = 0; i < rn.size(); i++)
        {
            rn[i]->setJointValueNoUpdate(jointValues[i]);
        }

        applyJointValuesNoLock();
    }

    VirtualRobot::BoundingBox Robot::getBoundingBox(bool collisionModel)
    {
        VirtualRobot::BoundingBox bbox;
        std::vector<RobotNodePtr> rn = getRobotNodes();

        for (size_t i = 0; i < rn.size(); i++)
        {
            if (collisionModel && rn[i]->getCollisionModel())
            {
                bbox.addPoints(rn[i]->getCollisionModel()->getBoundingBox());
            }
            else if (!collisionModel && rn[i]->getVisualization())
            {
                bbox.addPoints(rn[i]->getVisualization()->getBoundingBox());
            }
        }

        return bbox;
    }

    SensorPtr Robot::getSensor(const std::string& name)
    {
        std::vector<RobotNodePtr> rn = getRobotNodes();

        for (size_t i = 0; i < rn.size(); i++)
        {
            std::vector<SensorPtr> sensors = rn[i]->getSensors();

            for (std::vector<SensorPtr>::iterator it = sensors.begin(); it != sensors.end(); it++)
            {
                if ((*it)->getName() == name)
                {
                    return *it;
                }
            }
        }

        return SensorPtr();
    }

    std::vector<SensorPtr> Robot::getSensors()
    {
        std::vector<SensorPtr> result;
        std::vector<RobotNodePtr> rn = getRobotNodes();

        for (size_t i = 0; i < rn.size(); i++)
        {
            std::vector<SensorPtr> s = rn[i]->getSensors();

            if (s.size() > 0)
            {
                result.insert(result.end(), s.begin(), s.end());
            }
        }

        return result;
    }

    std::string Robot::toXML(const std::string& basePath,  const std::string& modelPath /*= "models"*/, bool storeEEF, bool storeRNS, bool storeSensors)
    {
        std::stringstream ss;
        ss << "<?xml version='1.0' encoding='UTF-8'?>" << endl << endl;
        ss << "<Robot Type='" << this->type << "' RootNode='" << this->getRootNode()->getName() << "'>" << endl << endl;
        std::vector<RobotNodePtr> nodes = getRobotNodes();

        for (size_t i = 0; i < nodes.size(); i++)
        {
            ss << nodes[i]->toXML(basePath, modelPath, storeSensors) << endl;
        }

        ss << endl;

        if (storeRNS)
        {
            std::vector<RobotNodeSetPtr> rns;
            this->getRobotNodeSets(rns);

            for (size_t i = 0; i < rns.size(); i++)
            {
                ss << rns[i]->toXML(1) << endl;
            }

            if (rns.size() > 0)
            {
                ss << endl;
            }
        }

        if (storeEEF)
        {
            std::vector<EndEffectorPtr> eefs = this->getEndEffectors();

            for (size_t i = 0; i < eefs.size(); i++)
            {
                ss << eefs[i]->toXML(1) << endl;
            }

            if (eefs.size() > 0)
            {
                ss << endl;
            }
        }

        ss << "</Robot>" << endl;

        return ss.str();
    }

} // namespace VirtualRobot

