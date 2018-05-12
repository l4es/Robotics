
#ifndef _MTPlanning_SCENERY_H_
#define _MTPlanning_SCENERY_H_

#include <string.h>
#include <time.h>

#include <MotionPlanning/Planner/PlanningThread.h>
#include <MotionPlanning/PostProcessing/PathProcessingThread.h>

#include <MotionPlanning/CSpace/CSpaceSampled.h>
#include <MotionPlanning/CSpace/CSpaceNode.h>
#include <MotionPlanning/Planner/MotionPlanner.h>
#include <MotionPlanning/Planner/BiRrt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>

#define ROBOT_DIM 3
#define SHORTEN_LOOP 600

using namespace VirtualRobot;
using namespace Saba;

class MTPlanningScenery
{
public:
    MTPlanningScenery();
    ~MTPlanningScenery();

    void loadRobotMTPlanning(bool bMultiCollisionCheckers);

    // switches the visualisation of the robot (colModel <-> fullModel)
    void setRobotModelShape(bool collisionModel);

    void buildScene();
    SoSeparator* getScene()
    {
        return sceneSep;
    }
    void reset();

    // if bMultiCollisionCheckers is set, more than one collision checker is used
    void buildPlanningThread(bool bMultiCollisionCheckers, int id);
    PathProcessingThreadPtr buildOptimizeThread(CSpaceSampledPtr cspace, CSpacePathPtr path);
    void startPlanning();
    void stopPlanning();

    void startOptimizing();
    void stopOptimizing();

    void checkPlanningThreads();
    void checkOptimizeThreads();

    void getThreadCount(int& nWorking, int& nIdle);
    void getOptimizeThreadCount(int& nWorking, int& nIdle);

    bool getPlannersStarted()
    {
        return this->plannersStarted;
    }
    bool getOptimizeStarted()
    {
        return this->optimizeStarted;
    }

    int getThreads();
    ///////////////////////////////////////////////////////////////////
    //Sequential planing
    //void loadRobotSTPlanning();
    //void plan(int index);
    //void optimizeSolution(int solutionIndex);
    //void showSolution(CRrtSolution *solToShow, int solutionIndex);

protected:

    void addBBCube(SoSeparator* result);

    void getRandomPos(float& x, float& y, float& z);

    std::string robotFilename;
    std::string colModel;
    std::string kinChainName;

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* obstSep;
    SoSeparator* startEndVisu;
    bool plannersStarted;
    bool optimizeStarted;

    std::vector<PlanningThreadPtr> planningThreads;
    std::vector<PathProcessingThreadPtr> optimizeThreads;
    std::vector<CSpaceSampledPtr> CSpaces;
    std::vector<RrtPtr> planners;
    std::vector<CSpacePathPtr> solutions;
    std::vector<CSpacePathPtr> optiSolutions;
    std::vector<SoSeparator*> visualisations;
    std::vector<RobotPtr> robots;
    //std::vector<RobotNodeSetPtr> colModelRobots;

    std::vector< Eigen::VectorXf > startPositions;
    std::vector< Eigen::VectorXf > goalPositions;


    SceneObjectSetPtr environment;
    ObstaclePtr environmentUnited;



    bool robotModelVisuColModel;


    std::string TCPName;

    ///////////////////////////////////////////////////////////////////
    //Sequential planing
    /*CRobot *m_pRobot;
    CRobotCollisionModelCollection *m_pColModelRobot;

    CSpaceSampled *m_pCSpace1;
    CSpaceSampled *m_pCSpace2;
    std::vector<CRrtBiPlanner*> m_vBiPlannersForSeq;
    std::vector<CRrtSolution*> solutionsForSeq;
    std::vector<CRrtSolution*> optiSolutionsForSeq;
    std::vector<SoSeparator*> m_vVisualizationsForSeq;
    */
};

#endif // _MTPlanning_SCENERY_H_
