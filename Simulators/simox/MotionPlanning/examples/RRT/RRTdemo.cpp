
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/SceneObjectSet.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <MotionPlanning/Saba.h>
#include <MotionPlanning/Planner/Rrt.h>
#include <MotionPlanning/Planner/BiRrt.h>
#include <MotionPlanning/Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/Qt/SoQt.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;
using namespace Saba;

#define USE_BIRRT

bool useColModel = false;
QWidget* win;

void show(SoNode* n)
{
    if (win == NULL)
    {
        printf("Could not create window.\n");
        exit(-3);
    }

    SoQtExaminerViewer* viewer = new SoQtExaminerViewer(win);

    // set the robot
    if (n)
    {
        viewer->setSceneGraph(n);
    }

    // register timer callback for animation and draw state updates
    /*
    SoSensorManager *sensor_mgr = SoDB::getSensorManager();
    SoTimerSensor *timer = new SoTimerSensor(TimerCallback, NULL);
    timer->setInterval(SbTime(0.02));
    sensor_mgr->insertTimerSensor(timer);
    */
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(true);
    viewer->setAntialiasing(true, 4);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer->setFeedbackVisibility(true);

    // show everything
    viewer->show();

    // start the mainloop
    SoQt::show(win);
    SoQt::mainLoop();

    // clean up
    delete viewer;
}

void startRRTVisualization()
{

    // create robot
    std::string filename("robots/examples/RrtDemo/Joint3.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);
    cout << "Loading 3DOF robot from " << filename << endl;
    RobotPtr robot = RobotIO::loadRobot(filename);

    if (!robot)
    {
        return;
    }


    float sampling_extend_stepsize = 0.04f;

    // create environment
    float posX = 0.0f, posY = -400.0f, posZ = 400.0f, sizeX = 100.0f, sizeY = 300.0f, sizeZ = 1000.0f;
    ObstaclePtr o = Obstacle::createBox(sizeX, sizeY, sizeZ);

    Eigen::Affine3f tmpT(Eigen::Translation3f(posX, posY, posZ));
    o->setGlobalPose(tmpT.matrix());


    // setup collision detection
    std::string colModelName("CollisionModel");
    SceneObjectSetPtr cms = robot->getRobotNodeSet(colModelName);
    CDManagerPtr cdm(new CDManager());
    cdm->addCollisionModel(cms);
    cdm->addCollisionModel(o);

    float planningTime = 0;
    int failed = 0;
    int loops = 1;
    bool ok;
    CSpaceSampledPtr cspace;
    std::string planningJoints("AllJoints");
    RobotNodeSetPtr planningNodes = robot->getRobotNodeSet(planningJoints);
#ifdef USE_BIRRT
    BiRrtPtr rrt;
#else
    RrtPtr rrt;
#endif
    Eigen::VectorXf start(3);
    start(0) = (float)M_PI / 4.0f;
    start(1) = (float)M_PI / 4.0f * 1.5f;
    start(2) = (float) - M_PI / 4.0f;

    Eigen::VectorXf goal(3);
    goal(0) = (float) - M_PI / 4.0f;
    goal(1) = (float)M_PI / 4.0f * 1.5f;
    goal(2) = (float) - M_PI / 4.0f;

    for (int i = 0; i < loops; i++)
    {
        // setup C-Space

        cspace.reset(new CSpaceSampled(robot, cdm, planningNodes));
        cspace->setSamplingSize(sampling_extend_stepsize);
        cspace->setSamplingSizeDCD(sampling_extend_stepsize);
        // setup planner


#ifdef USE_BIRRT
        rrt.reset(new BiRrt(cspace));
#else
        rrt.reset(new Rrt(cspace));
#endif
        rrt->setStart(start);
        rrt->setGoal(goal);

        clock_t startT = clock();
        ok = rrt->plan(true);
        clock_t endT = clock();

        float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        planningTime += diffClock;

        if (!ok)
        {
            failed++;
        }
    }

    planningTime /= (float)loops;
    cout << "Avg planning time: " << planningTime << endl;
    cout << "failed:" << failed << endl;

    if (!ok)
    {
        cout << "planning failed..." << endl;
        return;
    }

    CSpacePathPtr solution = rrt->getSolution();
    CSpaceTreePtr tree = rrt->getTree();



    robot->setJointValues(planningNodes, start);

    // display robot
    SoSeparator* sep = new SoSeparator();
    SceneObject::VisualizationType colModel = SceneObject::Full;

    boost::shared_ptr<CoinVisualization> visualization = robot->getVisualization<CoinVisualization>(colModel);
    SoNode* visualisationNode = NULL;

    if (visualization)
    {
        visualisationNode = visualization->getCoinVisualization();
    }

    sep->addChild(visualisationNode);

    // display obstacle
    VisualizationNodePtr visuObstacle = o->getVisualization();
    std::vector<VisualizationNodePtr> visus;
    visus.push_back(visuObstacle);
    boost::shared_ptr<CoinVisualization> visualizationO(new CoinVisualization(visus));
    SoNode* obstacleSoNode = visualizationO->getCoinVisualization();
    sep->addChild(obstacleSoNode);

    // show rrt visu
    boost::shared_ptr<CoinRrtWorkspaceVisualization> w(new CoinRrtWorkspaceVisualization(robot, cspace, "EndPoint"));
    w->addTree(tree);
#ifdef USE_BIRRT
    CSpaceTreePtr tree2 = rrt->getTree2();
    w->addTree(tree2);
#endif
    w->addCSpacePath(solution);
    w->addConfiguration(start, CoinRrtWorkspaceVisualization::eGreen, 3.0f);
    w->addConfiguration(goal, CoinRrtWorkspaceVisualization::eGreen, 3.0f);
    SoSeparator* sol = w->getCoinVisualization();
    sep->addChild(sol);

    show(sep);
}

int main(int argc, char** argv)
{
    SoDB::init();
    win = SoQt::init("RoboViewer", "RoboViewer");
    cout << " --- START --- " << endl;

    try
    {
        startRRTVisualization();
    }
    catch (VirtualRobot::VirtualRobotException v)
    {
        std::cout << "VirtualRobot Exception: " << v.what() << std::endl ;
    }
    catch (std::exception e)
    {
        std::cout << "Exception: " << e.what() << std::endl ;
    }
    catch (...)
    {
        ;
    }

    cout << " --- END --- " << endl;

    return 0;
}
