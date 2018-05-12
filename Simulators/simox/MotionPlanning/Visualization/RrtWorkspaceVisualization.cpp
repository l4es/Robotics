
#include "RrtWorkspaceVisualization.h"
#include "MotionPlanning/CSpace/CSpace.h"
#include "VirtualRobot/Nodes/RobotNode.h"

namespace Saba
{

    RrtWorkspaceVisualization::RrtWorkspaceVisualization(VirtualRobot::RobotPtr robot, CSpacePtr cspace, const std::string& TCPName)
    {
        this->robot = robot;
        this->cspace = cspace;
        SABA_ASSERT(robot);
        SABA_ASSERT(cspace);
        robotNodeSet = cspace->getRobotNodeSet();
        SABA_ASSERT(robotNodeSet);
        setTCPName(TCPName);
        init();
    }


    RrtWorkspaceVisualization::RrtWorkspaceVisualization(VirtualRobot::RobotPtr robot, VirtualRobot::RobotNodeSetPtr robotNodeSet, const std::string& TCPName)
    {
        this->robot = robot;
        cspace = CSpacePtr();
        SABA_ASSERT(robot);
        SABA_ASSERT(robotNodeSet);
        this->robotNodeSet = robotNodeSet;
        setTCPName(TCPName);
        init();
    }

    RrtWorkspaceVisualization::~RrtWorkspaceVisualization()
    {
    }

    void RrtWorkspaceVisualization::init()
    {
        setPathStyle();
        setTreeStyle();

        RenderColors red;
        red.nodeR = 1.0f;
        red.nodeG = 0;
        red.nodeB = 0;
        red.lineR = 0.5f;
        red.lineG = 0.5f;
        red.lineB = 0.5f;
        colors[RrtWorkspaceVisualization::eRed] = red;

        RenderColors blue;
        blue.nodeR = 0;
        blue.nodeG = 0;
        blue.nodeB = 1.0f;
        blue.lineR = 0.5f;
        blue.lineG = 0.5f;
        blue.lineB = 0.5f;
        colors[RrtWorkspaceVisualization::eBlue] = blue;

        RenderColors green;
        green.nodeR = 0;
        green.nodeG = 1.0f;
        green.nodeB = 0;
        green.lineR = 0.5f;
        green.lineG = 0.5f;
        green.lineB = 0.5f;
        colors[RrtWorkspaceVisualization::eGreen] = green;

        RenderColors custom;
        custom.nodeR = 1.0f;
        custom.nodeG = 1.0f;
        custom.nodeB = 0;
        custom.lineR = 0.5f;
        custom.lineG = 0.5f;
        custom.lineB = 0.5f;
        colors[RrtWorkspaceVisualization::eCustom] = custom;

    }


    void RrtWorkspaceVisualization::setPathStyle(float lineSize, float nodeSize, float renderComplexity)
    {
        pathLineSize = lineSize;
        pathNodeSize = nodeSize;
        pathRenderComplexity = renderComplexity;
    }

    void RrtWorkspaceVisualization::setTreeStyle(float lineSize, float nodeSize, float renderComplexity)
    {
        treeLineSize = lineSize;
        treeNodeSize = nodeSize;
        treeRenderComplexity = renderComplexity;
    }

    void RrtWorkspaceVisualization::setCustomColor(float nodeR, float nodeG, float nodeB, float lineR, float lineG, float lineB)
    {
        RenderColors custom;
        custom.nodeR = nodeR;
        custom.nodeG = nodeG;
        custom.nodeB = nodeB;
        custom.lineR = lineR;
        custom.lineG = lineG;
        custom.lineB = lineB;
        colors[RrtWorkspaceVisualization::eCustom] = custom;
    }


    void RrtWorkspaceVisualization::reset()
    {
    }

    void RrtWorkspaceVisualization::setTCPName(const std::string TCPName)
    {
        this->TCPName = TCPName;
        TCPNode = robot->getRobotNode(TCPName);

        if (!TCPNode)
        {
            VR_ERROR << "No node with name " << TCPName << " available in robot.." << endl;
        }
    }

    void RrtWorkspaceVisualization::colorizeTreeNodes(int status, ColorSet colorSet)
    {
        treeNodeStatusColor[status] = colorSet;
    }

    /*
    bool RrtWorkspaceVisualization::addConfig( const Eigen::VectorXf &c )
    {
        return false;
    }*/

} // namespace Saba
