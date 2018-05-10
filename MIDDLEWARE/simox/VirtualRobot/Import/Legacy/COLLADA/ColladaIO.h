#ifndef COLLADAIO_H
#define COLLADAIO_H

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Nodes/RobotNodeRevoluteFactory.h>
#include <VirtualRobot/Nodes/RobotNodeFixedFactory.h>
#include <VirtualRobot/Nodes/RobotNodePrismaticFactory.h>
#include <VirtualRobot/Nodes/PositionSensor.h>
#include "ColladaParser.h"
#include <boost/shared_ptr.hpp>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "../VirtualRobotImportExport.h"

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT ColladaIO
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        static VirtualRobot::RobotPtr loadRobot(const std::string& filename, float scaling = 1.0f);

    protected:

        static int nonameCounter;

        static VirtualRobot::PositionSensorPtr convertSensor(boost::shared_ptr<ColladaParser::NodeData> colladaNode, VirtualRobot::RobotNodePtr rn, float scaling);
        static VirtualRobot::RobotPtr convertRobot(ColladaParser::ModelType& colladaModel, float scaling);
        static VirtualRobot::RobotNodePtr convertNode(boost::shared_ptr<ColladaParser::NodeData> colladaNode, std::vector<VirtualRobot::RobotNodePtr>& allNodes, std::map< VirtualRobot::RobotNodePtr, std::vector<std::string> >& childrenMap, VirtualRobot::RobotPtr robo, float scaling, std::map<VirtualRobot::RobotNodePtr, float>& valueMap);
        static std::vector<std::string> getChildrenList(boost::shared_ptr<ColladaParser::NodeData> colladaNode);
        static Eigen::Matrix4f getTransformation(boost::shared_ptr<ColladaParser::NodeData> colladaNode, float scaling);
        static Eigen::Matrix4f getTransformation(std::vector<float>& trafo, float scaling);
        static Eigen::Matrix4f getTransformation(std::vector<std::vector<float> >& trafos, float scaling);
        static Eigen::Vector3f getAxis(boost::shared_ptr<ColladaParser::NodeData> colladaNode);
        static boost::shared_ptr<VirtualRobot::TriMeshModel> getMesh(boost::shared_ptr<ColladaParser::NodeData> colladaNode, const Eigen::Matrix4f& modelTrafo, float scaling);
        static bool addSceneGraph(boost::shared_ptr<VirtualRobot::TriMeshModel> triMesh, boost::shared_ptr<ColladaParser::SceneGraph> sceneGraph, const Eigen::Matrix4f& modelTrafo, float scaling);
        static bool addGeometry(boost::shared_ptr<VirtualRobot::TriMeshModel> triMesh, Eigen::Matrix4f& preModelTrafo, ColladaParser::SceneGraph::GeometryType& geom, float scaling);
    };

}

#endif
