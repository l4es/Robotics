
#include "SceneObjectSet.h"
#include "CollisionDetection/CollisionModel.h"
#include "CollisionDetection/CollisionChecker.h"
#include "Visualization//VisualizationNode.h"
#include "RobotNodeSet.h"
#include "Obstacle.h"


namespace VirtualRobot
{

    //----------------------------------------------------------------------
    // class SceneObjectSet constructor
    //----------------------------------------------------------------------
    SceneObjectSet::SceneObjectSet(const std::string& name, CollisionCheckerPtr colChecker)
    {
        this->name = name;

        this->colChecker = colChecker;

        if (!this->colChecker)
        {
            this->colChecker = CollisionChecker::getGlobalCollisionChecker();
        }
    }


    //----------------------------------------------------------------------
    // class SceneObjectSet destructor
    //----------------------------------------------------------------------
    SceneObjectSet::~SceneObjectSet()
    {
        destroyData();
    }



    void SceneObjectSet::destroyData()
    {
        sceneObjects.clear();
    }


    std::string SceneObjectSet::getName() const
    {
        return name;
    }


    bool SceneObjectSet::addSceneObject(SceneObjectPtr sceneObject)
    {
        if (!sceneObject)
        {
            VR_WARNING << "NULL data, in: " << name << endl;
            return false;
        }

        if (colChecker != sceneObject->getCollisionChecker())
        {
            VR_WARNING << " col model belongs to different instance of collision checker, in: " << name << endl;
            return false;
        }

        for (unsigned i = 0; i < sceneObjects.size(); i++)
            if (sceneObjects[i] == sceneObject)
            {
                VR_WARNING << "col model already added, in: " << name << endl;
                return false;
            }

        sceneObjects.push_back(sceneObject);
        return true;
    }



    bool SceneObjectSet::addSceneObjects(SceneObjectSetPtr sceneObjectSet)
    {
        if (!sceneObjectSet)
        {
            VR_WARNING << "NULL data, in: " << getName().c_str() << endl;
            return false;
        }

        if (colChecker != sceneObjectSet->getCollisionChecker())
        {
            VR_WARNING << "col model set belongs to different instance of collision checker, in: " << getName().c_str() << endl;
            return false;
        }

        std::vector< SceneObjectPtr > so = sceneObjectSet->getSceneObjects();

        for (size_t i = 0; i < so.size(); i++)
        {
            if (!addSceneObject(so[i]))
            {
                return false;
            }
        }

        return true;
    }

    bool SceneObjectSet::addSceneObjects(RobotNodeSetPtr robotNodeSet)
    {
        if (!robotNodeSet)
        {
            VR_WARNING << "NULL data, in: " << getName().c_str() << endl;
            return false;
        }

        std::vector< RobotNodePtr > robotNodes = robotNodeSet->getAllRobotNodes();
        return addSceneObjects(robotNodes);
    }

    bool SceneObjectSet::addSceneObjects(std::vector<RobotNodePtr> robotNodes)
    {
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
                    if (!addSceneObject(cm))
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    bool SceneObjectSet::removeSceneObject(SceneObjectPtr sceneObject)
    {
        if (!sceneObject)
        {
            return false;
        }

        bool found = false;

        for (std::vector< SceneObjectPtr >::iterator iter = sceneObjects.begin(); iter != sceneObjects.end(); iter++)
        {
            if (*iter == sceneObject)
            {
                found = true;
                sceneObjects.erase(iter);
                break;
            }
        }

        if (!found)
        {
            VR_WARNING << " col model not added, in: " << name << endl;
            return false;
        }

        return true;
    }
    /*
    void SceneObjectSet::GetAABB( SbBox3f& store_aabb )
    {
        SbViewportRegion vpreg;
        SoGetBoundingBoxAction bboxAction(vpreg);
        store_aabb.makeEmpty();
        SoNode *sep;
        for (std::vector<CollisionModel*>::iterator iter = sceneObjects.begin(); iter != sceneObjects.end(); iter++)
        {
            sep = (*iter)->GetIVModel();
            if (sep)
            {
                bboxAction.apply(sep);
                store_aabb.extendBy(bboxAction.getBoundingBox());
            }
        }
    }

    void SceneObjectSet::GetAABBs( std::vector<SbBox3f>& store_aabbs )
    {
        SbViewportRegion vpreg;
        SoGetBoundingBoxAction bboxAction(vpreg);
        store_aabbs.clear();
        SoNode *sep;
        for (std::vector<CollisionModel*>::iterator iter = sceneObjects.begin(); iter != sceneObjects.end(); iter++)
        {
            sep = (*iter)->GetIVModel();
            if (sep)
            {
                bboxAction.apply(sep);
                store_aabbs.push_back(bboxAction.getBoundingBox());
            }
        }
    }

    void SceneObjectSet::GetOOBBs( std::vector<SbXfBox3f>& store_oobbs )
    {
        SbViewportRegion vpreg;
        SoGetBoundingBoxAction bboxAction(vpreg);
        SoNode *sep;
        for (std::vector<CollisionModel*>::iterator iter = sceneObjects.begin(); iter != sceneObjects.end(); iter++)
        {
            sep = (*iter)->GetIVModel();
            if (sep)
            {
                bboxAction.apply(sep);
                //SbBox3f bbox = bboxAction.getBoundingBox();
                store_oobbs.push_back(bboxAction.getXfBoundingBox());
            }
        }
    }*/


    bool SceneObjectSet::getCurrentSceneObjectConfig(std::map< SceneObjectPtr, Eigen::Matrix4f >& storeConfig)
    {
        for (std::vector< SceneObjectPtr >::iterator iter = sceneObjects.begin(); iter != sceneObjects.end(); iter++)
        {
            storeConfig[(*iter)] = (*iter)->getGlobalPose();
        }

        return true;
    }

    bool SceneObjectSet::hasSceneObject(SceneObjectPtr sceneObject)
    {
        for (std::vector< SceneObjectPtr >::iterator iter = sceneObjects.begin(); iter != sceneObjects.end(); iter++)
        {
            if ((*iter) == sceneObject)
            {
                return true;
            }
        }

        return false;
    }

    std::vector< CollisionModelPtr > SceneObjectSet::getCollisionModels()
    {
        std::vector< CollisionModelPtr > result;

        for (size_t i = 0; i < sceneObjects.size(); i++)
        {
            if (sceneObjects[i]->getCollisionModel())
            {
                result.push_back(sceneObjects[i]->getCollisionModel());
            }
        }

        return result;
    }

    std::vector< SceneObjectPtr > SceneObjectSet::getSceneObjects()
    {
        return sceneObjects;
    }

    unsigned int SceneObjectSet::getSize() const
    {
        return (unsigned int)sceneObjects.size();
    }

    VirtualRobot::SceneObjectPtr SceneObjectSet::getSceneObject(unsigned int nr)
    {
        VR_ASSERT((nr < (unsigned int)sceneObjects.size()));
        return sceneObjects[nr];
    }

    std::string SceneObjectSet::toXML(int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += "\t";
        }

        ss << pre << "<SceneObjectSet name='" << name << "'>\n";

        for (size_t i = 0; i < sceneObjects.size(); i++)
        {
            ss << pre << t << "<SceneObject name='" << sceneObjects[i]->getName() << "'/>\n";
        }

        ss << pre << "</SceneObjectSet>\n";
        return ss.str();
    }

    VirtualRobot::SceneObjectSetPtr SceneObjectSet::clone(const std::string& newName /*= ""*/)
    {
        SceneObjectSetPtr result(new SceneObjectSet(newName, colChecker));

        for (size_t i = 0; i < sceneObjects.size(); i++)
        {
            result->addSceneObject(sceneObjects[i]);
        }

        return result;
    }


    VirtualRobot::SceneObjectSetPtr SceneObjectSet::clone(const std::string& newName, CollisionCheckerPtr newColChecker)
    {
        SceneObjectSetPtr result(new SceneObjectSet(newName, newColChecker));

        for (size_t i = 0; i < sceneObjects.size(); i++)
        {
            SceneObjectPtr o = sceneObjects[i]->clone(sceneObjects[i]->getName(), newColChecker);
            result->addSceneObject(o);
        }

        return result;
    }

    VirtualRobot::ObstaclePtr SceneObjectSet::createStaticObstacle(const std::string& name)
    {
        //VisualizationNodePtr visus(new VisualizationNode());
        std::vector<VisualizationNodePtr> visus;
        std::vector<CollisionModelPtr> cols;

        for (size_t i = 0; i < sceneObjects.size(); i++)
        {
            if (sceneObjects[i]->getVisualization())
            {
                visus.push_back(sceneObjects[i]->getVisualization());
            }

            if (sceneObjects[i]->getCollisionModel())
            {
                cols.push_back(sceneObjects[i]->getCollisionModel());
            }
        }

        VisualizationNodePtr unitedVisu = VisualizationNode::CreateUnitedVisualization(visus);
        CollisionModelPtr unitedColModel = CollisionModel::CreateUnitedCollisionModel(cols);
        SceneObject::Physics phys;
        return ObstaclePtr(new Obstacle(name, unitedVisu, unitedColModel, phys, colChecker));
    }

} // namespace VirtualRobot

