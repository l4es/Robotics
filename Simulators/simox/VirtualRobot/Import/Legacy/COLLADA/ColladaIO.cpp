#include "ColladaIO.h"
#include <boost/foreach.hpp>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/RobotNodeSet.h>

using namespace std;

namespace VirtualRobot
{


    int ColladaIO::nonameCounter = 0;

    RobotPtr ColladaIO::loadRobot(const std::string& filename, float scaling)
    {
        ColladaParser parser(filename);
        parser.parse();

        if (parser.getModel().empty())
        {
            VR_ERROR << "No model loaded..." << endl;
            return RobotPtr();
        }

        //cout << *(parser.getModel().begin()->second.get());
        //parser.print();
        ColladaParser::ModelType colladaModel = parser.getModel();
        VirtualRobot::RobotPtr robo = convertRobot(colladaModel, scaling);
        return  robo;
    }

    RobotPtr ColladaIO::convertRobot(ColladaParser::ModelType& colladaModel, float scaling)
    {
        THROW_VR_EXCEPTION_IF(colladaModel.empty(), "No data in collada file");
        std::string robotType = colladaModel.begin()->first;
        std::string robotName = robotType;
        VirtualRobot::RobotPtr robo(new VirtualRobot::LocalRobot(robotName, robotType));

        boost::shared_ptr<ColladaParser::NodeData> root = colladaModel.begin()->second;

        std::vector<VirtualRobot::RobotNodePtr> allNodes;
        std::map< VirtualRobot::RobotNodePtr, std::vector<std::string> > childrenMap;
        std::map< VirtualRobot::RobotNodePtr, float> valueMap;

        VirtualRobot::RobotNodePtr rootVR = convertNode(root, allNodes, childrenMap, robo, scaling, valueMap);



        VirtualRobot::RobotFactory::initializeRobot(robo, allNodes, childrenMap, rootVR);

        std::cout << "updating joint values\n";

        for (std::map< VirtualRobot::RobotNodePtr, float>::iterator it = valueMap.begin(); it != valueMap.end(); it++)
        {
            it->first->setJointValue(it->second);
            std::cout << it->second << std::endl;
        }

        // setup standard RNS
        std::vector<RobotNodePtr> jointsAll = robo->getRobotNodes();
        std::vector<RobotNodePtr> jointsRevolute;
        std::vector<RobotNodePtr> jointsPrismatic;
        std::vector<RobotNodePtr> rnVisu;

        for (size_t i = 0; i < jointsAll.size(); i++)
        {
            boost::shared_ptr<RobotNodePrismatic> rnPr = boost::dynamic_pointer_cast<RobotNodePrismatic>(jointsAll[i]);

            if (rnPr)
            {
                jointsPrismatic.push_back(jointsAll[i]);
            }

            boost::shared_ptr<RobotNodeRevolute> rnRe = boost::dynamic_pointer_cast<RobotNodeRevolute>(jointsAll[i]);

            if (rnRe)
            {
                jointsRevolute.push_back(jointsAll[i]);
            }

            if (jointsAll[i]->getVisualization())
            {
                rnVisu.push_back(jointsAll[i]);
            }
        }

        RobotNodeSetPtr rnAll = RobotNodeSet::createRobotNodeSet(robo, "All", jointsAll, RobotNodePtr(), RobotNodePtr(), true);

        if (!jointsRevolute.empty())
        {
            RobotNodeSetPtr rnRev = RobotNodeSet::createRobotNodeSet(robo, "Joints_Revolute", jointsRevolute, RobotNodePtr(), RobotNodePtr(), true);
        }

        if (!jointsPrismatic.empty())
        {
            RobotNodeSetPtr rnPri = RobotNodeSet::createRobotNodeSet(robo, "Joints_Prismatic", jointsPrismatic, RobotNodePtr(), RobotNodePtr(), true);
        }

        RobotNodeSetPtr rnVis = RobotNodeSet::createRobotNodeSet(robo, "Visualization", rnVisu, RobotNodePtr(), RobotNodePtr(), true);

        return robo;
    }


    Eigen::Vector3f ColladaIO::getAxis(boost::shared_ptr<ColladaParser::NodeData> colladaNode)
    {
        THROW_VR_EXCEPTION_IF(!colladaNode || colladaNode->axis.size() != 3, "Expecting 3dim axis");

        Eigen::Vector3f ax;
        ax << colladaNode->axis[0], colladaNode->axis[1], colladaNode->axis[2];
        return ax;
    }

    Eigen::Matrix4f ColladaIO::getTransformation(std::vector<float>& trafo, float scaling)
    {
        THROW_VR_EXCEPTION_IF(trafo.size() != 3 && trafo.size() != 4 && trafo.size() != 16, "Expecting a 3 or 4 dim vector or a 16 element matrix");
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        if (trafo.size() == 3)
        {
            transform.block(0, 3, 3, 1) << trafo[0]*scaling, trafo[1]*scaling, trafo[2]*scaling;
        }
        else if (trafo.size() == 4)
        {
            //old format: degrees, new format: radian
            //        Eigen::AngleAxisf aa(trafo[3]/180.0f*(float)M_PI,Eigen::Vector3f(trafo[0], trafo[1], trafo[2]));
            Eigen::AngleAxisf aa(trafo[3], Eigen::Vector3f(trafo[0], trafo[1], trafo[2]));
            transform.block(0, 0, 3, 3) = aa.toRotationMatrix();

        }
        else
        {
            //row major
            transform << trafo[0], trafo[1], trafo[2], trafo[3] * scaling,
                      trafo[4], trafo[5], trafo[6], trafo[7] * scaling,
                      trafo[8], trafo[9], trafo[10], trafo[11] * scaling,
                      trafo[12], trafo[13], trafo[14], trafo[15];
            //column major
            //        for (size_t i = 0; i < trafo.size(); i++) {
            //            transform(i) = trafo[i];
            //        }
        }

        cout << transform << endl;
        return transform;
    }

    Eigen::Matrix4f ColladaIO::getTransformation(boost::shared_ptr<ColladaParser::NodeData> colladaNode, float scaling)
    {
        THROW_VR_EXCEPTION_IF(!colladaNode, "Null data");
        return getTransformation(colladaNode->transformations, scaling);
    }

    Eigen::Matrix4f ColladaIO::getTransformation(std::vector<std::vector<float> >& trafos, float scaling)
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        BOOST_FOREACH(std::vector<float>& trafo, trafos)
        {
            Eigen::Matrix4f t = getTransformation(trafo, scaling);
            transform = transform * t;
        }
        return transform;
    }

    std::vector<std::string> ColladaIO::getChildrenList(boost::shared_ptr<ColladaParser::NodeData> colladaNode)
    {
        std::vector<std::string> result;
        BOOST_FOREACH(boost::shared_ptr<ColladaParser::NodeData> child, colladaNode->children)
        {
            std::string name = child->name;

            if (name.empty())
            {
                std::stringstream ss;
                ss << "robot_node_" << nonameCounter;
                name = ss.str();
            }

            // oops: this may cause bugs, since the nonameCounter might be increased differently!!!!
            result.push_back(name + "_Transformation");
        }
        return result;
    }

    bool ColladaIO::addGeometry(boost::shared_ptr<VirtualRobot::TriMeshModel> triMesh, Eigen::Matrix4f& preModelTrafo, ColladaParser::SceneGraph::GeometryType& geom, float scaling)
    {
        bool perVertexNormals = false;

        if (!triMesh)
        {
            VR_ERROR << "Null data" << endl;
            return false;
        }

        size_t startIndx = triMesh->vertices.size();
        size_t startIndxColor = triMesh->colors.size();
        size_t startIndxMaterial = triMesh->materials.size();
        size_t startIndxNormals = triMesh->normals.size();

        for (size_t i = 0; i < geom.vertices.size(); i++)
        {
            Eigen::Vector4f v;

            if (geom.vertices[i].size() != 3)
            {
                VR_ERROR << "Vertex " << i << ": array dim !=3" << endl;
            }
            else
            {
                v << geom.vertices[i][0]*scaling, geom.vertices[i][1]*scaling, geom.vertices[i][2]*scaling, 1.0f;
                v = preModelTrafo * v;
                triMesh->addVertex(v.head(3));
            }
        }

        for (size_t i = 0; i < geom.colors.size(); i++)
        {
            if (geom.colors[i].size() != 3)
            {
                VR_ERROR << "Color " << i << ": array dim !=3" << endl;
            }
            else
            {
                Eigen::Vector4f c(geom.colors[i][0], geom.colors[i][1], geom.colors[i][2], 0.f);
                triMesh->addColor(c);
            }
        }

        if (perVertexNormals)
        {
            for (size_t i = 0; i < geom.normals.size(); i++)
            {
                if (geom.normals[i].size() != 3)
                {
                    VR_ERROR << "Normal " << i << ": array dim !=3" << endl;
                }
                else
                {
                    Eigen::Vector3f c(geom.normals[i][0], geom.normals[i][1], geom.normals[i][2]);
                    c = preModelTrafo.block(0, 0, 3, 3) * c;
                    triMesh->addNormal(c);
                }
            }
        }

        for (size_t i = 0; i < geom.materials.size(); i++)
        {
            VirtualRobot::VisualizationFactory::PhongMaterial mat;
            mat.transparency = geom.materials[i].transparency;
            mat.shininess = geom.materials[i].shininess / 128.0f;
            mat.reflectivity = geom.materials[i].reflectivity;
            mat.refractionIndex = geom.materials[i].refractionIndex;

            if (geom.materials[i].ambient.size() != 3 && geom.materials[i].ambient.size() != 4)
            {
                mat.ambient.r = 0;
                mat.ambient.g = 0;
                mat.ambient.b = 0;
                VR_ERROR << "Material ambient " << i << ": wrong array dimension" << endl;
            }
            else
            {
                mat.ambient.r = geom.materials[i].ambient[0];
                mat.ambient.g = geom.materials[i].ambient[1];
                mat.ambient.b = geom.materials[i].ambient[2];
            }

            if (geom.materials[i].diffuse.size() != 3 && geom.materials[i].diffuse.size() != 4)
            {
                mat.diffuse.r = 0;
                mat.diffuse.g = 0;
                mat.diffuse.b = 0;
                VR_ERROR << "Material diffuse " << i << ": wrong array dimension" << endl;
            }
            else
            {
                mat.diffuse.r = geom.materials[i].diffuse[0];
                mat.diffuse.g = geom.materials[i].diffuse[1];
                mat.diffuse.b = geom.materials[i].diffuse[2];
            }

            if (geom.materials[i].emission.size() != 3 && geom.materials[i].emission.size() != 4)
            {
                mat.emission.r = 0;
                mat.emission.g = 0;
                mat.emission.b = 0;
                VR_ERROR << "Material emission " << i << ": wrong array dimension" << endl;
            }
            else
            {
                mat.emission.r = geom.materials[i].emission[0];
                mat.emission.g = geom.materials[i].emission[1];
                mat.emission.b = geom.materials[i].emission[2];
            }

            if (geom.materials[i].reflective.size() != 3 && geom.materials[i].reflective.size() != 4)
            {
                mat.reflective.r = 0;
                mat.reflective.g = 0;
                mat.reflective.b = 0;
                VR_ERROR << "Material reflective " << i << ": wrong array dimension" << endl;
            }
            else
            {
                mat.reflective.r = geom.materials[i].reflective[0];
                mat.reflective.g = geom.materials[i].reflective[1];
                mat.reflective.b = geom.materials[i].reflective[2];
            }

            if (geom.materials[i].specular.size() != 3 && geom.materials[i].specular.size() != 4)
            {
                mat.specular.r = 0;
                mat.specular.g = 0;
                mat.specular.b = 0;
                VR_ERROR << "Material specular " << i << ": wrong array dimension" << endl;
            }
            else
            {
                mat.specular.r = geom.materials[i].specular[0];
                mat.specular.g = geom.materials[i].specular[1];
                mat.specular.b = geom.materials[i].specular[2];
            }


            if (geom.materials[i].transparent.size() != 3 && geom.materials[i].transparent.size() != 4)
            {
                mat.transparent.r = 0;
                mat.transparent.g = 0;
                mat.transparent.b = 0;
                VR_ERROR << "Material transparent " << i << ": wrong array dimension" << endl;
            }
            else
            {
                mat.transparent.r = geom.materials[i].transparent[0];
                mat.transparent.g = geom.materials[i].transparent[1];
                mat.transparent.b = geom.materials[i].transparent[2];
            }

            triMesh->addMaterial(mat);
        }

        //cout << "Adding " << geom.triangles.size() << "triangles" << endl;
        for (size_t i = 0; i < geom.triangles.size(); i++)
        {
            // some checks
            if (geom.triangles[i].size() < 3)
            {
                VR_ERROR << "Triangle " << i << ": array dim = " << geom.triangles[i].size() << "(<3)" << endl;
                continue;
            }

            for (size_t j = 0; j < geom.triangles[i].size(); j += 3)
            {
                if (j + 2 >= geom.triangles[i].size())
                {
                    VR_ERROR << "Corrupted triangle vector. Size:" << geom.triangles[i].size() << endl;
                    break;
                }

                bool fOK = true;

                /*if (geom.triangles[i][j]<0 || geom.triangles[i][j]>=geom.vertices.size())
                {
                    VR_ERROR << "geom Triangle data is corrupt: geom.triangles["<<i<<"]["<<j<<"]:" << geom.triangles[i][j] << ". Vertices vector size:" << geom.vertices.size() << endl;
                    fOK = false;
                }*/
                if (fOK)
                {
                    unsigned int i1 = geom.triangles[i][j] + startIndx;
                    unsigned int i2 = geom.triangles[i][j + 1] + startIndx;
                    unsigned int i3 = geom.triangles[i][j + 2] + startIndx;

                    if (i1 >= triMesh->vertices.size() || i2 >= triMesh->vertices.size() || i3 >= triMesh->vertices.size())
                    {
                        VR_ERROR << "Corrupted triangle indexes:" << i1 << "," << i2 << "," << i3 << ". trimesh->Vertices vector size:" << triMesh->vertices.size() << endl;
                        continue;
                    }

                    // check for degenerated triangles
                    float d1 = (triMesh->vertices[i1] - triMesh->vertices[i2]).norm();
                    float d2 = (triMesh->vertices[i1] - triMesh->vertices[i3]).norm();
                    float d3 = (triMesh->vertices[i3] - triMesh->vertices[i2]).norm();
                    float eps = 1e-8f;

                    if (d1 < eps || d2 < eps || d3 < eps)
                    {
                        continue;
                    }

                    MathTools::TriangleFace t;
                    t.set(i1, i2, i3);

                    if (perVertexNormals)
                    {
                        t.setNormal(geom.triangleNormals[i][j] + startIndxNormals, geom.triangleNormals[i][j + 1] + startIndxNormals, geom.triangleNormals[i][j + 2] + startIndxNormals);
                    }
                    else
                    {
                        t.normal = triMesh->CreateNormal(triMesh->vertices[i1], triMesh->vertices[i2], triMesh->vertices[i3]);
                    }

                    if (geom.triangleColors.size() != 0)
                    {
                        unsigned int iColor1 = geom.triangleColors[i][j] + startIndxColor;
                        unsigned int iColor2 = geom.triangleColors[i][j + 1] + startIndxColor;
                        unsigned int iColor3 = geom.triangleColors[i][j + 2] + startIndxColor;
                        t.setColor(iColor1, iColor2, iColor3);
                    }

                    if (geom.triangleMaterial.size() != 0)
                    {
                        t.setMaterial(geom.triangleMaterial[i] + startIndxColor);
                    }


                    triMesh->addFace(t);
                }
            }
        }

        return true;
    }


    //TODO: This fails when several meshes are connected to the same node!!
    bool ColladaIO::addSceneGraph(boost::shared_ptr<VirtualRobot::TriMeshModel> triMesh,  boost::shared_ptr<ColladaParser::SceneGraph> sceneGraph, const Eigen::Matrix4f& modelTrafo, float scaling)
    {
        if (!triMesh || !sceneGraph)
        {
            VR_ERROR << "Null data" << endl;
            return false;
        }

        Eigen::Matrix4f preModelTrafo = modelTrafo * getTransformation(sceneGraph->transformations, scaling);
        cout << "pMT: " << preModelTrafo << endl;

        for (size_t g = 0; g < sceneGraph->geometries.size(); g++)
        {
            addGeometry(triMesh, preModelTrafo, sceneGraph->geometries[g], scaling);
        }

        for (size_t g = 0; g < sceneGraph->children.size(); g++)
        {
            addSceneGraph(triMesh, sceneGraph->children[g], preModelTrafo, scaling);
        }

        return true;
    }

    boost::shared_ptr<VirtualRobot::TriMeshModel> ColladaIO::getMesh(boost::shared_ptr<ColladaParser::NodeData> colladaNode, const Eigen::Matrix4f& modelTrafo, float scaling)
    {
        boost::shared_ptr<VirtualRobot::TriMeshModel> triMeshPtr;

        if (colladaNode->sceneGraph && (colladaNode->sceneGraph->geometries.size() > 0 || colladaNode->sceneGraph->children.size() > 0))
        {
            triMeshPtr.reset(new VirtualRobot::TriMeshModel());
            addSceneGraph(triMeshPtr, colladaNode->sceneGraph, modelTrafo, scaling);

            //check trimesh faces
            unsigned int nrV = triMeshPtr->vertices.size();

            for (size_t i = 0; i < triMeshPtr->faces.size(); i++)
            {
                if (triMeshPtr->faces[i].id1 >= nrV || triMeshPtr->faces[i].id2 >= nrV || triMeshPtr->faces[i].id3 >= nrV)
                {
                    VR_ERROR << "Face " << i << " is corrupted: " << triMeshPtr->faces[i].id1 << "," << triMeshPtr->faces[i].id2 << "," << triMeshPtr->faces[i].id3 << ". nr vert:" << nrV << endl;
                    triMeshPtr->faces.erase(triMeshPtr->faces.begin() + i, triMeshPtr->faces.begin() + i + 1);
                    i--;
                }
            }
        }

        return triMeshPtr;
    }


    PositionSensorPtr ColladaIO::convertSensor(boost::shared_ptr<ColladaParser::NodeData> colladaNode, VirtualRobot::RobotNodePtr rn, float scaling)
    {
        if (!colladaNode)
        {
            return PositionSensorPtr();
        }

        std::string name = colladaNode->name;

        if (name.empty())
        {
            name = rn->getName();
            name += std::string("_sensor");
        }

        Eigen::Matrix4f rnTrafo = getTransformation(colladaNode, scaling);
        boost::shared_ptr<TriMeshModel> m = getMesh(colladaNode, Eigen::Matrix4f::Identity() /*rnTrafo.inverse()*/, scaling);
        VirtualRobot::VisualizationNodePtr visualizationNode;

        if (m && m->vertices.size() && m->faces.size() > 0)
        {
            VisualizationFactoryPtr f = VisualizationFactory::first(NULL);

            if (f)
            {
                Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
                visualizationNode = f->createTriMeshModelVisualization(m, pose);
            }
        }

        PositionSensorPtr s(new PositionSensor(rn, name, visualizationNode, rnTrafo));
        return s;
    }


    RobotNodePtr ColladaIO::convertNode(boost::shared_ptr<ColladaParser::NodeData> colladaNode, std::vector<VirtualRobot::RobotNodePtr>& allNodes, std::map< VirtualRobot::RobotNodePtr, std::vector<std::string> >& childrenMap, RobotPtr robo, float scaling, std::map<VirtualRobot::RobotNodePtr, float>& valueMap)
    {
        THROW_VR_EXCEPTION_IF(!colladaNode, "NULL collada node");
        VirtualRobot::RobotNodeFactoryPtr revoluteNodeFactory = VirtualRobot::RobotNodeFactory::fromName(VirtualRobot::RobotNodeRevoluteFactory::getName(), NULL);
        VirtualRobot::RobotNodeFactoryPtr fixedNodeFactory = VirtualRobot::RobotNodeFactory::fromName(VirtualRobot::RobotNodeFixedFactory::getName(), NULL);
        VirtualRobot::RobotNodeFactoryPtr prismaticNodeFactory = VirtualRobot::RobotNodeFactory::fromName(VirtualRobot::RobotNodePrismaticFactory::getName(), NULL);

        Eigen::Matrix4f idMatrix = Eigen::Matrix4f::Identity();
        Eigen::Vector3f idVec3 = Eigen::Vector3f::Zero();
        std::string name = colladaNode->name;

        if (name.empty())
        {
            VR_WARNING << "Node without name!!!" << endl;
            std::stringstream ss;
            ss << "robot_node_" << nonameCounter;
            nonameCounter++;
            name = ss.str();
        }

        // TRANSFORMATION
        std::string robotNodeTrafoName = name + "_Transformation";

        // PrejointTransform is not required --> if the ColladaSceneGraph is correct)
        Eigen::Matrix4f preJointTransform = getTransformation(colladaNode, scaling);

        VirtualRobot::RobotNodePtr robotNodeTrafo = fixedNodeFactory->createRobotNode(robo, robotNodeTrafoName, VirtualRobot::VisualizationNodePtr(), VirtualRobot::CollisionModelPtr(), 0,
                0, 0, preJointTransform, idVec3, idVec3);
        robo->registerRobotNode(robotNodeTrafo);
        allNodes.push_back(robotNodeTrafo);



        // create visu model
        cout << "Reading visual scene\npJT: " << preJointTransform << endl;
        boost::shared_ptr<TriMeshModel> m = getMesh(colladaNode, preJointTransform.inverse(), scaling);
        VirtualRobot::VisualizationNodePtr visualizationNode;
        VirtualRobot::CollisionModelPtr colModel;

        if (m && m->vertices.size() > 0 && m->faces.size() > 0)
        {
            cout << "creating visu for " << name << endl;
            VisualizationFactoryPtr f = VisualizationFactory::first(NULL);

            if (f)
            {
                Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
                visualizationNode = f->createTriMeshModelVisualization(m, pose);
                colModel.reset(new CollisionModel(visualizationNode));
            }
        }

        // check for joint info
        VirtualRobot::RobotNodePtr robotNodeJoint;
        float jointOffset = 0.0;

        if (colladaNode->active)
        {
            if (colladaNode->jointType == ColladaParser::eRevolute)
            {
                // Revolute JOINT
                float jointLimitLow = colladaNode->min / 180.0f * (float)M_PI;
                float jointLimitHigh = colladaNode->max / 180.0f * (float)M_PI;
                std::cout << "###### " << jointOffset << std::endl;
                Eigen::Vector3f axis = getAxis(colladaNode);
                //jointOffset=(-1)*colladaNode->value /180.0f * (float)M_PI;
                robotNodeJoint = revoluteNodeFactory->createRobotNode(robo, name, visualizationNode, colModel,
                                 jointLimitLow, jointLimitHigh, jointOffset, idMatrix, axis, idVec3);
                valueMap[robotNodeJoint] = colladaNode->value / 180.0f * (float)M_PI;
                robo->registerRobotNode(robotNodeJoint);
                allNodes.push_back(robotNodeJoint);
            }
            else if (colladaNode->jointType == ColladaParser::ePrismatic)
            {
                // Prismatic JOINT
                float jointLimitLow = colladaNode->min * scaling;
                float jointLimitHigh = colladaNode->max * scaling;
                //jointOffset=(-1)*colladaNode->value *1000;
                Eigen::Vector3f axis = getAxis(colladaNode);
                robotNodeJoint = prismaticNodeFactory->createRobotNode(robo, name, visualizationNode, colModel,
                                 jointLimitLow, jointLimitHigh, jointOffset, idMatrix, idVec3, axis);
                valueMap[robotNodeJoint] = colladaNode->value * scaling;
                robo->registerRobotNode(robotNodeJoint);
                allNodes.push_back(robotNodeJoint);

                // setup model scaling
                if (visualizationNode)
                {
                    /*
                    boost::shared_ptr<VirtualRobot::RobotNodePrismatic> rnPr = boost::dynamic_pointer_cast<RobotNodePrismatic>(robotNodeJoint);
                    Eigen::Vector3f one = Eigen::Vector3f::Ones();
                    float l = preJointTransform.block(0, 3, 3, 1).norm();
                    if (l > 0)
                        one /= l;
                    rnPr->setVisuScaleFactor(one);
                    */
                }
            }
        }

        if (!robotNodeJoint && visualizationNode)
        {
            // ceate fixed model to hold visualization
            std::string robotNode2Name = name + "_Model";

            robotNodeJoint = fixedNodeFactory->createRobotNode(robo, robotNode2Name, visualizationNode, colModel, 0, 0, 0, idMatrix,
                             idVec3, idVec3);
            robo->registerRobotNode(robotNodeJoint);
            allNodes.push_back(robotNodeJoint);
        }

        // VISU MODEL
        /*
        std::string robotNode2Name = name + "_Model";
        boost::shared_ptr<TriMeshModel> m = getMesh(colladaNode, preJointTransform.inverse(), scaling);
        VirtualRobot::VisualizationNodePtr visualizationNode;
        VirtualRobot::CollisionModelPtr colModel;


        if (m && m->vertices.size()>0)
        {
            VisualizationFactoryPtr f = VisualizationFactory::first(NULL);
            if (f)
            {
                Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
                visualizationNode = f->createTriMeshModelVisualization(m, pose);
                colModel.reset(new CollisionModel(visualizationNode));
            }
        }

        VirtualRobot::RobotNodePtr robotNodeModel = fixedNodeFactory->createRobotNode(robo, robotNode2Name, visualizationNode, colModel, 0, 0, 0, idMatrix,
            idVec3, idVec3);
        robo->registerRobotNode(robotNodeModel);
        allNodes.push_back(robotNodeModel);
        */

        if (colladaNode->active && robotNodeJoint)
        {
            // setup trafo->joint children
            std::vector<std::string> childrenTrafo;
            childrenTrafo.push_back(robotNodeJoint->getName());
            childrenMap[robotNodeTrafo] = childrenTrafo;

            // setup joint->nextNodes children
            std::vector<std::string> childrenJoint = getChildrenList(colladaNode);
            childrenMap[robotNodeJoint] = childrenJoint;

        }
        else
        {
            // setup trafo->nextNodes children
            std::vector<std::string> childrenTrafo = getChildrenList(colladaNode);
            childrenMap[robotNodeTrafo] = childrenTrafo;
        }

        /*else
        {
            childrenTrafo.push_back(robotNodeModel->getName());
        }
        childrenMap[robotNodeTrafo] = childrenTrafo;
        */
        /*
        if (colladaNode->active)
        {
            std::vector<std::string> childrenJoint;
            childrenJoint.push_back(robotNodeModel->getName());
            childrenMap[robotNodeJoint] = childrenJoint;
        }*/
        /*
        std::vector<std::string> childrenModel = getChildrenList(colladaNode);
        childrenMap[robotNodeModel] = childrenModel;
        */

        // check for sensors
        for (size_t i = 0; i < colladaNode->sensors.size(); i++)
        {
            boost::shared_ptr<ColladaParser::NodeData> n = colladaNode->sensors[i];
            VirtualRobot::PositionSensorPtr s = convertSensor(n, robotNodeJoint, scaling);

            if (s && robotNodeJoint)
            {
                robotNodeJoint->registerSensor(s);
            }
        }

        BOOST_FOREACH(boost::shared_ptr<ColladaParser::NodeData> child, colladaNode->children)
        {
            convertNode(child, allNodes, childrenMap, robo, scaling, valueMap);
        }

        return robotNodeTrafo;
    }

}
