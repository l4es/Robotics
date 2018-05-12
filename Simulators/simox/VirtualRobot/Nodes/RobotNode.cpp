
#include "RobotNode.h"
#include "../VirtualRobot.h"
#include "../VirtualRobotException.h"
#include "../Robot.h"
#include "../RobotNodeSet.h"
#include "../Visualization/VisualizationFactory.h"
#include "../Visualization/Visualization.h"
#include "../Visualization/TriMeshModel.h"
#include "../XML/BaseIO.h"
#include <cmath>
#include <iomanip>

#include <algorithm>





#include <Eigen/Core>

using namespace boost;

namespace VirtualRobot
{

    RobotNode::RobotNode(RobotWeakPtr rob,
                         const std::string& name,
                         float jointLimitLo,
                         float jointLimitHi,
                         VisualizationNodePtr visualization,
                         CollisionModelPtr collisionModel,
                         float jointValueOffset,
                         const SceneObject::Physics& p,
                         CollisionCheckerPtr colChecker,
                         RobotNodeType type)
        : SceneObject(name, visualization, collisionModel, p, colChecker)


    {
        nodeType = type;
        maxVelocity = -1.0f;
        maxAcceleration = -1.0f;
        maxTorque = -1.0f;
        robot = rob;
        this->jointLimitLo = jointLimitLo;
        this->jointLimitHi = jointLimitHi;
        this->jointValueOffset = jointValueOffset;
        localTransformation = Eigen::Matrix4f::Identity();
        //postJointTransformation = Eigen::Matrix4f::Identity();
        optionalDHParameter.isSet = false;
        //globalPosePostJoint = Eigen::Matrix4f::Identity();
        jointValue = 0.0f;
    }


    RobotNode::~RobotNode()
    {
        // not needed here
        // when robot is destroyed all references to this RobotNode are also destroyed
        //RobotPtr rob = robot.lock();
        //if (rob)
        //  rob->deregisterRobotNode(static_pointer_cast<RobotNodePtr>(shared_from_this()));
    }


    bool RobotNode::initialize(SceneObjectPtr parent, const std::vector<SceneObjectPtr>& children)
    {
        RobotPtr rob = robot.lock();
        THROW_VR_EXCEPTION_IF(!rob, "Could not init RobotNode without robot");

        // robot
        if (!rob->hasRobotNode(static_pointer_cast<RobotNode>(shared_from_this())))
        {
            rob->registerRobotNode(static_pointer_cast<RobotNode>(shared_from_this()));
        }

        // update visualization of coordinate systems
        if (visualizationModel && visualizationModel->hasAttachedVisualization("CoordinateSystem"))
        {
            VisualizationNodePtr v = visualizationModel->getAttachedVisualization("CoordinateSystem");
            // not needed any more!
            // this is a little hack: The globalPose is used to set the "local" position of the attached Visualization:
            // Since the attached visualizations are already positioned at the global pose of the visualizationModel,
            // we just need the local postJointTransform
            //v->setGlobalPose(postJointTransformation);
        }

        checkValidRobotNodeType();

        for (size_t i = 0; i < sensors.size(); i++)
        {
            sensors[i]->initialize(shared_from_this());
        }

        return SceneObject::initialize(parent, children);
    }

    void RobotNode::checkValidRobotNodeType()
    {
        switch (nodeType)
        {
            case Generic:
                return;
                break;

            case Joint:
                THROW_VR_EXCEPTION_IF(visualizationModel, "No visualization models allowed in JointNodes");
                THROW_VR_EXCEPTION_IF(collisionModel, "No collision models allowed in JointNodes");
                //THROW_VR_EXCEPTION_IF(postJointTransformation != Eigen::Matrix4f::Identity() , "No postJoint transformations allowed in JointNodes");
                break;

            case Body:
                //THROW_VR_EXCEPTION_IF(postJointTransformation != Eigen::Matrix4f::Identity() , "No transformations allowed in BodyNodes");
                THROW_VR_EXCEPTION_IF(localTransformation != Eigen::Matrix4f::Identity() , "No transformations allowed in BodyNodes");

                break;

            case Transform:
                THROW_VR_EXCEPTION_IF(visualizationModel, "No visualization models allowed in TransformationNodes");
                THROW_VR_EXCEPTION_IF(collisionModel, "No collision models allowed in TransformationNodes");
                break;

            default:
                VR_ERROR << "RobotNodeType nyi..." << endl;

        }

    }

    RobotPtr RobotNode::getRobot() const
    {
        RobotPtr result(robot);
        return result;
    }

    void RobotNode::setJointValue(float q)
    {
        RobotPtr r = getRobot();
        VR_ASSERT(r);
        WriteLockPtr lock = r->getWriteLock();
        setJointValueNoUpdate(q);
        updatePose();
    }

    void RobotNode::setJointValueNotInitialized(float q)
    {
        VR_ASSERT_MESSAGE((!boost::math::isnan(q) && !boost::math::isinf(q)) , "Not a valid number...");

        //std::cout << "######## Setting Joint to: " << q << " degrees" << std::endl;

        if (q < jointLimitLo)
        {
            q = jointLimitLo;
        }

        if (q > jointLimitHi)
        {
            q = jointLimitHi;
        }

        jointValue = q;
    }
    void RobotNode::setJointValueNoUpdate(float q)
    {
        VR_ASSERT_MESSAGE(initialized, "Not initialized");
        VR_ASSERT_MESSAGE((!boost::math::isnan(q) && !boost::math::isinf(q)) , "Not a valid number...");

        //std::cout << "######## Setting Joint to: " << q << " degrees" << std::endl;

        if (q < jointLimitLo)
        {
            q = jointLimitLo;
        }

        if (q > jointLimitHi)
        {
            q = jointLimitHi;
        }

        jointValue = q;
    }

    void RobotNode::updateTransformationMatrices()
    {
        if (this->getParent())
        {
            updateTransformationMatrices(this->getParent()->getGlobalPose());
        }
        else
        {
            // check for root
            RobotPtr r = getRobot();

            if (r && r->getRootNode() == shared_from_this())
            {
                updateTransformationMatrices(r->getGlobalPose());
            }
            else
            {
                updateTransformationMatrices(Eigen::Matrix4f::Identity());
            }
        }
    }

    void RobotNode::updateTransformationMatrices(Eigen::Matrix4f& newLocalTransformation) {
        this->localTransformation = newLocalTransformation;

    }

    void RobotNode::updateTransformationMatrices(const Eigen::Matrix4f& parentPose)
    {
        this->globalPose = parentPose * getLocalTransformation();

        //globalPosePostJoint = this->globalPose*getPostJointTransformation();
    }


    void RobotNode::updatePose(bool updateChildren)
    {
        THROW_VR_EXCEPTION_IF(!initialized, "Not initialized");

        updateTransformationMatrices();

        // update collision and visualization model and children
        SceneObject::updatePose(updateChildren);

        // apply propagated joint values
        if (propagatedJointValues.size() > 0)
        {
            RobotPtr r = robot.lock();
            std::map< std::string, float>::iterator it = propagatedJointValues.begin();

            while (it != propagatedJointValues.end())
            {
                RobotNodePtr rn = r->getRobotNode(it->first);

                if (!rn)
                {
                    VR_WARNING << "Could not propagate joint value from " << name << " to " << it->first << " because dependent joint does not exist...";
                }
                else
                {
                    rn->setJointValue(jointValue * it->second);
                }

                it++;
            }
        }
    }

    void RobotNode::updatePose(const Eigen::Matrix4f& globalPose, bool updateChildren)
    {
        THROW_VR_EXCEPTION_IF(!initialized, "Not initialized");

        updateTransformationMatrices(globalPose);

        // update collision and visualization model and children
        SceneObject::updatePose(updateChildren);
    }

    void RobotNode::collectAllRobotNodes(std::vector< RobotNodePtr >& storeNodes)
    {
        storeNodes.push_back(static_pointer_cast<RobotNode>(shared_from_this()));

        std::vector< SceneObjectPtr > children = this->getChildren();

        for (size_t i = 0; i < children.size(); i++)
        {
            RobotNodePtr n = dynamic_pointer_cast<RobotNode>(children[i]);

            if (n)
            {
                n->collectAllRobotNodes(storeNodes);
            }
        }
    }

    float RobotNode::getJointValue() const
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        return jointValue;
    }

    void RobotNode::respectJointLimits(float& jointValue) const
    {
        if (jointValue < jointLimitLo)
        {
            jointValue = jointLimitLo;
        }

        if (jointValue > jointLimitHi)
        {
            jointValue = jointLimitHi;
        }
    }

    bool RobotNode::checkJointLimits(float jointValue, bool verbose) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        bool res = true;

        if (jointValue < jointLimitLo)
        {
            res = false;
        }

        if (jointValue > jointLimitHi)
        {
            res = false;
        }

        if (!res && verbose)
        {
            VR_INFO << "Joint: " << getName() << ": joint value (" << jointValue << ") is out of joint boundaries (lo:" << jointLimitLo << ", hi: " << jointLimitHi << ")" << endl;
        }

        return res;
    }
    void RobotNode::setGlobalPose(const Eigen::Matrix4f& pose)
    {
        THROW_VR_EXCEPTION("Use setJointValues to control the position of a RobotNode");
    }

    void RobotNode::print(bool printChildren, bool printDecoration) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();

        if (printDecoration)
        {
            cout << "******** RobotNode ********" << endl;
        }

        cout << "* Name: " << name << endl;
        cout << "* Parent: ";
        SceneObjectPtr p = this->getParent();

        if (p)
        {
            cout << p->getName() << endl;
        }
        else
        {
            cout << " -- " << endl;
        }

        cout << "* Children: ";

        if (this->getChildren().size() == 0)
        {
            cout << " -- " << endl;
        }

        for (unsigned int i = 0; i < this->getChildren().size(); i++)
        {
            cout << this->getChildren()[i]->getName() << ", ";
        }

        cout << endl;

        physics.print();

        cout << "* Limits: Lo:" << jointLimitLo << ", Hi:" << jointLimitHi << endl;
        std::cout << "* max velocity " << maxVelocity  << " [m/s]" << std::endl;
        std::cout << "* max acceleration " << maxAcceleration  << " [m/s^2]" << std::endl;
        std::cout << "* max torque " << maxTorque  << " [Nm]" << std::endl;
        cout << "* jointValue: " << this->getJointValue() << ", jointValueOffset: " << jointValueOffset << endl;

        if (optionalDHParameter.isSet)
        {
            cout << "* DH parameters: ";
            cout << " a:" << optionalDHParameter.aMM() << ", d:" << optionalDHParameter.dMM() << ", alpha:" << optionalDHParameter.alphaRadian() << ", theta:" << optionalDHParameter.thetaRadian() << endl;
        }
        else
        {
            cout << "* DH parameters: not specified." << endl;
        }

        cout << "* visualization model: " << endl;

        if (visualizationModel)
        {
            visualizationModel->print();
        }
        else
        {
            cout << "  No visualization model" << endl;
        }

        cout << "* collision model: " << endl;

        if (collisionModel)
        {
            collisionModel->print();
        }
        else
        {
            cout << "  No collision model" << endl;
        }

        if (initialized)
        {
            cout << "* initialized: true" << endl;
        }
        else
        {
            cout << "* initialized: false" << endl;
        }

        {
            // scope1
            std::ostringstream sos;
            sos << std::setiosflags(std::ios::fixed);
            sos << "* localTransformation:" << endl << localTransformation << endl;
            sos << "* globalPose:" << endl << getGlobalPose() << endl;
            cout << sos.str();
        } // scope1

        if (printDecoration)
        {
            cout << "******** End RobotNode ********" << endl;
        }

        if (printChildren)
        {
            std::vector< SceneObjectPtr > children = this->getChildren();

            for (unsigned int i = 0; i < children.size(); i++)
            {
                children[i]->print(true, true);
            }
        }
    }

    RobotNodePtr RobotNode::clone(RobotPtr newRobot, bool cloneChildren, RobotNodePtr initializeWithParent, CollisionCheckerPtr colChecker, float scaling)
    {
        ReadLockPtr lock = getRobot()->getReadLock();

        if (!newRobot)
        {
            VR_ERROR << "Attempting to clone RobotNode for invalid robot";
            return RobotNodePtr();
        }

        std::vector< std::string > clonedChildrenNames;

        VisualizationNodePtr clonedVisualizationNode;

        if (visualizationModel)
        {
            clonedVisualizationNode = visualizationModel->clone(true, scaling);
        }

        CollisionModelPtr clonedCollisionModel;

        if (collisionModel)
        {
            clonedCollisionModel = collisionModel->clone(colChecker, scaling);
        }

        RobotNodePtr result = _clone(newRobot, clonedVisualizationNode, clonedCollisionModel, colChecker, scaling);

        if (!result)
        {
            VR_ERROR << "Cloning failed.." << endl;
            return result;
        }

        if (cloneChildren)
        {
            std::vector< SceneObjectPtr > children = this->getChildren();

            for (size_t i = 0; i < children.size(); i++)
            {
                RobotNodePtr n = dynamic_pointer_cast<RobotNode>(children[i]);

                if (n)
                {
                    RobotNodePtr c = n->clone(newRobot, true, RobotNodePtr(), colChecker, scaling);

                    if (c)
                    {
                        result->attachChild(c);
                    }
                }
                else
                {
                    SensorPtr s =  dynamic_pointer_cast<Sensor>(children[i]);

                    if (s)
                    {
                        // performs registering and initialization
                        SensorPtr c = s->clone(result, scaling);
                    }
                    else
                    {
                        SceneObjectPtr so = children[i]->clone(children[i]->getName(), colChecker, scaling);

                        if (so)
                        {
                            result->attachChild(so);
                        }
                    }
                }
            }
        }

        result->setMaxVelocity(maxVelocity);
        result->setMaxAcceleration(maxAcceleration);
        result->setMaxTorque(maxTorque);


        std::map< std::string, float>::iterator it = propagatedJointValues.begin();

        while (it != propagatedJointValues.end())
        {
            result->propagateJointValue(it->first, it->second);
            it++;
        }


        newRobot->registerRobotNode(result);

        if (initializeWithParent)
        {
            result->initialize(initializeWithParent);
        }

        return result;
    }


    float RobotNode::getJointLimitLo()
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        return jointLimitLo;
    }

    float RobotNode::getJointLimitHi()
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        return jointLimitHi;
    }

    bool RobotNode::isTranslationalJoint() const
    {
        return false;
    }

    bool RobotNode::isRotationalJoint() const
    {
        return false;
    }


    void RobotNode::showCoordinateSystem(bool enable, float scaling, std::string* text, const std::string& visualizationType)
    {
        if (!enable && !visualizationModel)
        {
            return;    // nothing to do
        }

        if (!ensureVisualization(visualizationType))
        {
            return;
        }

        std::string coordName = name;

        if (text)
        {
            coordName = *text;
        }

        if (visualizationModel->hasAttachedVisualization("CoordinateSystem"))
        {
            visualizationModel->detachVisualization("CoordinateSystem");
        }

        if (enable)
        {
            VisualizationFactoryPtr visualizationFactory;

            if (visualizationType.empty())
            {
                visualizationFactory = VisualizationFactory::first(NULL);
            }
            else
            {
                visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
            }

            if (!visualizationFactory)
            {
                VR_WARNING << "No visualization factory for name " << visualizationType << endl;
                return;
            }

            // create coord visu
            VisualizationNodePtr visualizationNode = visualizationFactory->createCoordSystem(scaling, &coordName);

            // not needed any more
            // this is a little hack: The globalPose is used to set the "local" position of the attached Visualization:
            // Since the attached visualizations are already positioned at the global pose of the visualizationModel,
            // we just need the local postJointTransform
            if (visualizationNode)
            {
                //visualizationNode->setGlobalPose(postJointTransformation);
                visualizationModel->attachVisualization("CoordinateSystem", visualizationNode);
            }
        }
    }

    void RobotNode::showStructure(bool enable, const std::string& visualizationType)
    {
        ReadLockPtr lock = getRobot()->getReadLock();

        if (!enable && !visualizationModel)
        {
            return;    // nothing to do
        }

        if (!ensureVisualization(visualizationType))
        {
            return;
        }

        std::stringstream ss;
        ss << getName() << "_RobotNodeStructurePre";
        std::string attachName1 = ss.str();
        std::string attachName2("RobotNodeStructureJoint");
        std::string attachName3("RobotNodeStructurePost");
        SceneObjectPtr par = getParent();
        RobotNodePtr parRN = dynamic_pointer_cast<RobotNode>(par);

        // need to add "pre" visualization to parent node!
        if (parRN && parRN->getVisualization())
        {
            parRN->getVisualization()->detachVisualization(attachName1);
        }
        else
        {
            visualizationModel->detachVisualization(attachName1);
        }

        visualizationModel->detachVisualization(attachName2);
        visualizationModel->detachVisualization(attachName3);

        if (enable)
        {
            VisualizationFactoryPtr visualizationFactory;

            if (visualizationType.empty())
            {
                visualizationFactory = VisualizationFactory::first(NULL);
            }
            else
            {
                visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
            }

            if (!visualizationFactory)
            {
                VR_WARNING << "No visualization factory for name " << visualizationType << endl;
                return;
            }

            // create visu
            Eigen::Matrix4f i = Eigen::Matrix4f::Identity();

            if (!localTransformation.isIdentity())
            {
                VisualizationNodePtr visualizationNode1;

                if (parRN && parRN->getVisualization())
                {
                    // add to parent node (pre joint trafo moves with parent!)
                    //visualizationNode1 = visualizationFactory->createLine(parRN->postJointTransformation, parRN->postJointTransformation*localTransformation);
                    visualizationNode1 = visualizationFactory->createLine(Eigen::Matrix4f::Identity(), localTransformation);

                    if (visualizationNode1)
                    {
                        parRN->getVisualization()->attachVisualization(attachName1, visualizationNode1);
                    }
                }
                else
                {
                    visualizationNode1 = visualizationFactory->createLine(localTransformation.inverse(), i);

                    if (visualizationNode1)
                    {
                        visualizationModel->attachVisualization(attachName1, visualizationNode1);
                    }
                }
            }

            VisualizationNodePtr visualizationNode2 = visualizationFactory->createSphere(5.0f);

            if (visualizationNode2)
            {
                visualizationModel->attachVisualization(attachName2, visualizationNode2);
            }
        }
    }

    std::vector<RobotNodePtr> RobotNode::getAllParents(RobotNodeSetPtr rns)
    {
        std::vector<RobotNodePtr> result;

        if (!rns)
        {
            return result;
        }

        std::vector<RobotNodePtr> rn = rns->getAllRobotNodes();

        for (unsigned int i = 0; i < rn.size(); i++)
        {
            if (rn[i]->hasChild(static_pointer_cast<SceneObject>(shared_from_this()), true))
            {
                result.push_back(rn[i]);
            }
        }

        return result;
    }

    void RobotNode::setJointLimits(float lo, float hi)
    {
        jointLimitLo = lo;
        jointLimitHi = hi;
    }

    void RobotNode::setMaxTorque(float maxTo)
    {
        maxTorque = maxTo;
    }

    void RobotNode::setMaxAcceleration(float maxAcc)
    {
        maxAcceleration = maxAcc;
    }

    void RobotNode::setMaxVelocity(float maxVel)
    {
        maxVelocity = maxVel;
    }

    float RobotNode::getMaxVelocity()
    {
        return maxVelocity;
    }

    float RobotNode::getMaxAcceleration()
    {
        return maxAcceleration;
    }

    float RobotNode::getMaxTorque()
    {
        return maxTorque;
    }

    void RobotNode::updateVisualizationPose(const Eigen::Matrix4f& globalPose, float jointValue, bool updateChildren)
    {
        updateVisualizationPose(globalPose, updateChildren);
        this->jointValue = jointValue;
    }
    void RobotNode::updateVisualizationPose(const Eigen::Matrix4f& globalPose, bool updateChildren)
    {
        // check if we are a root node
        SceneObjectPtr parent = getParent();
        RobotPtr rob = getRobot();

        if (!parent || parent == rob)
        {
            if (rob && rob->getRootNode() == static_pointer_cast<RobotNode>(shared_from_this()))
            {
                Eigen::Matrix4f gpPre = getLocalTransformation().inverse() * globalPose;
                rob->setGlobalPose(gpPre, false);
            }
            else
            {
                VR_WARNING << "INTERNAL ERROR: getParent==robot but getRoot!=this ?! " << endl;
            }
        }

        this->globalPose = globalPose;

        // update collision and visualization model and children
        SceneObject::updatePose(updateChildren);
    }

    Eigen::Matrix4f RobotNode::getGlobalPose() const
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        return globalPose;
    }

    Eigen::Matrix4f RobotNode::getPoseInRootFrame() const
    {
        RobotPtr r = getRobot();
        ReadLockPtr lock = r->getReadLock();
        return r->getRootNode()->toLocalCoordinateSystem(globalPose);
    }

    Eigen::Vector3f RobotNode::getPositionInRootFrame() const
    {
        RobotPtr r = getRobot();
        ReadLockPtr lock = r->getReadLock();
        return r->getRootNode()->toLocalCoordinateSystemVec(globalPose.block(0,3,3,1));
    }

    RobotNode::RobotNodeType RobotNode::getType()
    {
        return nodeType;
    }

    void RobotNode::propagateJointValue(const std::string& jointName, float factor /*= 1.0f*/)
    {
        if (factor == 0.0f)
        {
            propagatedJointValues.erase(jointName);
        }
        else
        {
            propagatedJointValues[jointName] = factor;
        }
    }

    SensorPtr RobotNode::getSensor(const std::string& name) const
    {
        for (size_t i = 0; i < sensors.size(); i++)
        {
            if (sensors[i]->getName() == name)
            {
                return sensors[i];
            }
        }

        THROW_VR_EXCEPTION("No sensor with name" << name << " registerd at robot node " << getName());
        return SensorPtr();
    }

    bool RobotNode::hasSensor(const std::string& name) const
    {
        for (size_t i = 0; i < sensors.size(); i++)
        {
            if (sensors[i]->getName() == name)
            {
                return true;
            }
        }

        return false;
    }

    bool RobotNode::registerSensor(SensorPtr sensor)
    {
        if (!this->hasChild(sensor))
        {
            sensors.push_back(sensor);
            this->attachChild(sensor);
        }

        // if we are already initialized, be sure the sensor is also intialized
        if (initialized)
        {
            sensor->initialize(shared_from_this());
        }

        return true;
    }

    std::vector<SensorPtr> RobotNode::getSensors() const
    {
        return sensors;
    }

    std::string RobotNode::toXML(const std::string& basePath, const std::string& modelPathRelative /*= "models"*/, bool storeSensors)
    {
        std::stringstream ss;
        ss << "\t<RobotNode name='" << name << "'>" << endl;

        if (!localTransformation.isIdentity())
        {
            ss << "\t\t<Transform>" << endl;
            ss << BaseIO::toXML(localTransformation, "\t\t\t");
            ss << "\t\t</Transform>" << endl;
        }

        ss << _toXML(modelPathRelative);

        if (physics.isSet())
        {
            ss << physics.toXML(2);
        }

        boost::filesystem::path pBase(basePath);

        if (visualizationModel && visualizationModel->getTriMeshModel() && visualizationModel->getTriMeshModel()->faces.size() > 0)
        {
            std::string visuFile = getFilenameReplacementVisuModel();

            boost::filesystem::path pModel(modelPathRelative);
            boost::filesystem::path modelDirComplete = boost::filesystem::operator/(pBase, pModel);
            boost::filesystem::path fn(visuFile);
            boost::filesystem::path modelFileComplete = boost::filesystem::operator/(modelDirComplete, fn);

            ss << visualizationModel->toXML(pBase.string(), modelFileComplete.string(), 2);
        }

        if (collisionModel && collisionModel->getTriMeshModel() && collisionModel->getTriMeshModel()->faces.size() > 0)
        {
            std::string colFile = getFilenameReplacementColModel();
            boost::filesystem::path pModel(modelPathRelative);
            boost::filesystem::path modelDirComplete = boost::filesystem::operator/(pBase, pModel);
            boost::filesystem::path fn(colFile);
            boost::filesystem::path modelFileComplete = boost::filesystem::operator/(modelDirComplete, fn);
            ss << collisionModel->toXML(pBase.string(), modelFileComplete.string(), 2);
        }

        if (storeSensors)
        {
            for (size_t i = 0; i < sensors.size(); i++)
            {
                ss << sensors[i]->toXML(modelPathRelative, 2);
            }
        }

        std::vector<SceneObjectPtr> children = this->getChildren();

        for (size_t i = 0; i < children.size(); i++)
        {
            // check if child is a RobotNode
            RobotNodePtr crn = boost::dynamic_pointer_cast<RobotNode>(children[i]);

            if (crn)
            {
                ss << "\t\t<Child name='" << children[i]->getName() << "'/>" << endl;
            }
        }

        ss << "\t</RobotNode>" << endl << endl;
        return ss.str();
    }


} // namespace VirtualRobot
