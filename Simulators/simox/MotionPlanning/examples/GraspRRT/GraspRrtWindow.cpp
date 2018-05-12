
#include "GraspRrtWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/Obstacle.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/RobotIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "MotionPlanning/CSpace/CSpaceSampled.h"
#include "MotionPlanning/Planner/Rrt.h"
#include "MotionPlanning/Planner/GraspRrt.h"
#include "MotionPlanning/PostProcessing/ShortcutProcessor.h"
#include <MotionPlanning/Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h>
#include <QFileDialog>
#include <Eigen/Geometry>
#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoMatrixTransform.h>

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 200.0f;

GraspRrtWindow::GraspRrtWindow(const std::string& sceneFile, const std::string& sConf, const std::string& goalObject,
                               const std::string& rns, const std::string& rnsB, const std::string& eefName, const std::string& eefNameB,
                               const std::string& colModelRob1, const std::string& colModelRob1B, const std::string& colModelRob2, const std::string& colModelRob2B,
                               const std::string& colModelEnv, Qt::WFlags flags)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    this->sceneFile = sceneFile;

    allSep = new SoSeparator;
    allSep->ref();
    sceneFileSep = new SoSeparator;
    graspsSep = new SoSeparator;
    rrtSep = new SoSeparator;

    allSep->addChild(sceneFileSep);
    allSep->addChild(graspsSep);
    allSep->addChild(rrtSep);

    planSetA.rns = rns;
    planSetA.eef = eefName;
    planSetA.colModelRob1 = colModelRob1;
    planSetA.colModelRob2 = colModelRob2;
    planSetA.colModelEnv = colModelEnv;

    planSetB.rns = rnsB;
    planSetB.eef = eefNameB;
    planSetB.colModelRob1 = colModelRob1B;
    planSetB.colModelRob2 = colModelRob2B;
    planSetB.colModelEnv = colModelEnv;
    setupUI();

    loadScene();



    selectPlanSet(0);

    selectStart(sConf);
    selectTargetObject(goalObject);



    if (sConf != "")
    {
        UI.comboBoxStart->setEnabled(false);
    }

    if (goalObject != "")
    {
        UI.comboBoxGoal->setEnabled(false);
    }

    if (rns != "")
    {
        UI.comboBoxRNS->setEnabled(false);
    }

    //if (eefName!="")
    //  UI.comboBoxEEF->setEnabled(false);
    if (colModelRob1 != "")
    {
        UI.comboBoxColModelRobot->setEnabled(false);
    }

    if (colModelRob2 != "")
    {
        UI.comboBoxColModelRobotStatic->setEnabled(false);
    }

    //if (colModelEnv!="")
    //  UI.comboBoxColModelEnv->setEnabled(false);

    viewer->viewAll();

    SoSensorManager* sensor_mgr = SoDB::getSensorManager();
    SoTimerSensor* timer = new SoTimerSensor(timerCB, this);
    timer->setInterval(SbTime(TIMER_MS / 1000.0f));
    sensor_mgr->insertTimerSensor(timer);
}


GraspRrtWindow::~GraspRrtWindow()
{
    allSep->unref();
}


void GraspRrtWindow::timerCB(void* data, SoSensor* sensor)
{
    GraspRrtWindow* ikWindow = static_cast<GraspRrtWindow*>(data);
    ikWindow->redraw();
}


void GraspRrtWindow::setupUI()
{
    UI.setupUi(this);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(true);

    viewer->setAntialiasing(true, 4);

    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(allSep);
    viewer->viewAll();

    UI.radioButtonSolution->setChecked(true);

    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(loadSceneWindow()));
    connect(UI.checkBoxShowSolution, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxShowSolutionOpti, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxShowRRT, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxGrasps, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.pushButtonPlan, SIGNAL(clicked()), this, SLOT(plan()));
    connect(UI.horizontalSliderPos, SIGNAL(sliderMoved(int)), this, SLOT(sliderSolution(int)));
    connect(UI.radioButtonSolution, SIGNAL(clicked()), this, SLOT(solutionSelected()));
    connect(UI.radioButtonSolutionOpti, SIGNAL(clicked()), this, SLOT(solutionSelected()));

    connect(UI.comboBoxStart, SIGNAL(activated(int)), this, SLOT(selectStart(int)));
    connect(UI.comboBoxGoal, SIGNAL(activated(int)), this, SLOT(selectTargetObject(int)));
    connect(UI.comboBoxRNS, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));
    connect(UI.comboBoxEEF, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));
    connect(UI.comboBoxColModelRobot, SIGNAL(activated(int)), this, SLOT(selectColModelRobA(int)));
    connect(UI.comboBoxColModelRobotStatic, SIGNAL(activated(int)), this, SLOT(selectColModelRobB(int)));
    connect(UI.comboBoxColModelEnv, SIGNAL(activated(int)), this, SLOT(selectColModelEnv(int)));

    connect(UI.pushButtonGraspPose, SIGNAL(clicked()), this, SLOT(testGraspPose()));

    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));

}




void GraspRrtWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void GraspRrtWindow::buildVisu()
{
    sceneFileSep->removeAllChildren();

    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    if (scene)
    {
        visualization = scene->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = NULL;

        if (visualization)
        {
            visualisationNode = visualization->getCoinVisualization();
        }

        if (visualisationNode)
        {
            sceneFileSep->addChild(visualisationNode);
        }
    }

    graspsSep->removeAllChildren();

    if (UI.checkBoxGrasps->isChecked())
    {
        SoSeparator* eefVisu = CoinVisualizationFactory::CreateEndEffectorVisualization(eef);

        for (size_t i = 0; i < grasps.size(); i++)
        {

            Eigen::Matrix4f m = grasps[i]->getTcpPoseGlobal(targetObject->getGlobalPose());
            SoSeparator* sep1 = new SoSeparator();
            SoMatrixTransform* mt = CoinVisualizationFactory::getMatrixTransformScaleMM2M(m);
            sep1->addChild(mt);
            sep1->addChild(eefVisu);
            graspsSep->addChild(sep1);
        }
    }

    /*if (UI.checkBoxStartGoal->isChecked())
    {
        if (robotStart)
        {
            SoNode *st = CoinVisualizationFactory::getCoinVisualization(robotStart,colModel);
            if (st)
                startGoalSep->addChild(st);
        }
        if (robotGoal)
        {
            SoNode *go = CoinVisualizationFactory::getCoinVisualization(robotGoal,colModel);
            if (go)
                startGoalSep->addChild(go);
        }
    }*/
    buildRRTVisu();

    redraw();
}

int GraspRrtWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void GraspRrtWindow::quit()
{
    std::cout << "GraspRrtWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void GraspRrtWindow::loadSceneWindow()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Scene File"), QString(), tr("XML Files (*.xml)"));

    if (fi == "")
    {
        return;
    }

    sceneFile = std::string(fi.toAscii());
    loadScene();
}

void GraspRrtWindow::loadScene()
{
    robot.reset();
    scene = SceneIO::loadScene(sceneFile);

    if (!scene)
    {
        VR_ERROR << " no scene ..." << endl;
        return;
    }

    std::vector< RobotPtr > robots = scene->getRobots();

    if (robots.size() != 1)
    {
        VR_ERROR << "Need exactly 1 robot" << endl;
        return;
    }

    robot = robots[0];
    //robotStart = robot->clone("StartConfig");
    //robotGoal = robot->clone("GoalConfig");

    // setup start Config combo box
    configs = scene->getRobotConfigs(robot);

    if (configs.size() < 1)
    {
        VR_ERROR << "Need at least 1 Robot Configurations" << endl;
        return;
    }

    UI.comboBoxStart->clear();

    for (size_t i = 0; i < configs.size(); i++)
    {
        QString qtext = configs[i]->getName().c_str();
        UI.comboBoxStart->addItem(qtext);

    }

    UI.comboBoxStart->setCurrentIndex(0);
    selectStart(0);

    // setup target object combo box
    obstacles = scene->getObstacles();

    if (obstacles.size() < 1)
    {
        VR_ERROR << "Need at least 1 Obstacle (target object)" << endl;
        return;
    }

    UI.comboBoxGoal->clear();

    for (size_t i = 0; i < obstacles.size(); i++)
    {
        QString qtext = obstacles[i]->getName().c_str();
        UI.comboBoxGoal->addItem(qtext);

    }

    UI.comboBoxGoal->setCurrentIndex(0);
    //selectTargetObject(0);

    // steup scene objects (col models env)
    std::vector<SceneObjectSetPtr> soss = scene->getSceneObjectSets();
    UI.comboBoxColModelEnv->clear();
    QString qtext;

    for (size_t i = 0; i < soss.size(); i++)
    {
        qtext = soss[i]->getName().c_str();
        UI.comboBoxColModelEnv->addItem(qtext);
    }

    qtext = "<none>";
    UI.comboBoxColModelEnv->addItem(qtext);

    //setup eefs
    if (!robot->hasEndEffector(planSetA.eef))
    {
        VR_ERROR << "EEF with name " << planSetA.eef << " not known?!" << endl;
        return;
    }

    UI.comboBoxEEF->clear();
    qtext = planSetA.eef.c_str();
    UI.comboBoxEEF->addItem(qtext);

    if (robot->hasEndEffector(planSetB.eef))
    {
        qtext = planSetB.eef.c_str();
        UI.comboBoxEEF->addItem(qtext);
    }

    /*std::vector<EndEffectorPtr> eefs = robot->getEndEffectors();
    for (size_t i=0; i < eefs.size(); i++)
    {
        qtext = planSetA.eef.c_str();
        UI.comboBoxEEF->addItem(qtext);
    }*/

    // Setup robot node sets and col models
    std::vector<RobotNodeSetPtr> rnss = robot->getRobotNodeSets();
    UI.comboBoxColModelRobot->clear();
    UI.comboBoxColModelRobotStatic->clear();
    UI.comboBoxRNS->clear();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        qtext = rnss[i]->getName().c_str();
        UI.comboBoxColModelRobot->addItem(qtext);
        UI.comboBoxColModelRobotStatic->addItem(qtext);
        UI.comboBoxRNS->addItem(qtext);
    }

    qtext = "<none>";
    UI.comboBoxColModelRobot->addItem(qtext);
    UI.comboBoxColModelRobotStatic->addItem(qtext);
    robot->setThreadsafe(false);
    buildVisu();
}

void GraspRrtWindow::selectStart(const std::string& conf)
{
    for (size_t i = 0; i < configs.size(); i++)
    {
        if (configs[i]->getName() == conf)
        {
            selectStart(i);
            UI.comboBoxStart->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No configuration with name <" << conf << "> found..." << endl;
}

void GraspRrtWindow::selectTargetObject(const std::string& conf)
{
    for (size_t i = 0; i < obstacles.size(); i++)
    {
        if (obstacles[i]->getName() == conf)
        {
            selectTargetObject(i);
            UI.comboBoxGoal->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No obstacle with name <" << conf << "> found..." << endl;
}

void GraspRrtWindow::selectRNS(const std::string& rns)
{
    if (!robot)
    {
        return;
    }

    std::vector< RobotNodeSetPtr > rnss = robot->getRobotNodeSets();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        if (rnss[i]->getName() == rns)
        {
            selectRNS(i);
            UI.comboBoxRNS->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No rns with name <" << rns << "> found..." << endl;
}

void GraspRrtWindow::selectEEF(const std::string& eefName)
{
    if (!robot)
    {
        return;
    }

    if (eefName == planSetA.eef && UI.comboBoxEEF->count() > 0)
    {
        //selectEEF(0);
        UI.comboBoxEEF->setCurrentIndex(0);
        this->eef = robot->getEndEffector(eefName);
    }
    else if (eefName == planSetB.eef && UI.comboBoxEEF->count() > 1)
    {
        //selectEEF(1);
        UI.comboBoxEEF->setCurrentIndex(1);
        this->eef = robot->getEndEffector(eefName);
    }
    else
    {
        VR_ERROR << "No eef with name <" << eefName << "> found..." << endl;
        return;
    }

    /*std::vector< EndEffectorPtr > eefs = robot->getEndEffectors();
    for (size_t i=0;i<eefs.size();i++)
    {
        if (eefs[i]->getName()==eefName)
        {
            selectEEF(i);
            UI.comboBoxEEF->setCurrentIndex(i);
            return;
        }
    }
    VR_ERROR << "No eef with name <" << eefName << "> found..." << endl;*/
}

void GraspRrtWindow::selectColModelRobA(const std::string& colModel)
{
    if (!robot)
    {
        return;
    }

    std::vector< RobotNodeSetPtr > rnss = robot->getRobotNodeSets();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        if (rnss[i]->getName() == colModel)
        {
            selectColModelRobA(i);
            UI.comboBoxColModelRobot->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No col model set with name <" << colModel << "> found..." << endl;
}
void GraspRrtWindow::selectColModelRobB(const std::string& colModel)
{
    if (!robot)
    {
        return;
    }

    std::vector< RobotNodeSetPtr > rnss = robot->getRobotNodeSets();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        if (rnss[i]->getName() == colModel)
        {
            selectColModelRobB(i);
            UI.comboBoxColModelRobotStatic->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No col model set with name <" << colModel << "> found..." << endl;
}
void GraspRrtWindow::selectColModelEnv(const std::string& colModel)
{
    if (!scene)
    {
        return;
    }

    std::vector< SceneObjectSetPtr > rnss = scene->getSceneObjectSets();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        if (rnss[i]->getName() == colModel)
        {
            selectColModelEnv(i);
            UI.comboBoxColModelEnv->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No scene object set with name <" << colModel << "> found..." << endl;
}

void GraspRrtWindow::selectStart(int nr)
{
    if (nr < 0 || nr >= (int)configs.size())
    {
        return;
    }

    //if (robotStart)
    //  configs[nr]->applyToRobot(robotStart);
    if (robot)
    {
        robot->setJointValues(configs[nr]);
    }

    //configs[nr]->setJointValues();
    if (rns)
    {
        rns->getJointValues(startConfig);
    }
}

void GraspRrtWindow::selectTargetObject(int nr)
{
    if (nr < 0 || nr >= (int)obstacles.size())
    {
        return;
    }

    //if (robotGoal)
    //  configs[nr]->applyToRobot(robotGoal);
    //configs[nr]->setJointValues();
    //if (rns)
    //  rns->getJointValues(goalConfig);
    targetObject = obstacles[nr];
    graspQuality.reset(new GraspStudio::GraspQualityMeasureWrenchSpace(targetObject));
    int points = 400;
#ifdef _DEBUG
    points = 100;
#endif
    graspQuality->calculateOWS(points);
}

void GraspRrtWindow::selectRNS(int nr)
{
    this->rns.reset();

    if (!robot)
    {
        return;
    }

    std::vector< RobotNodeSetPtr > rnss = robot->getRobotNodeSets();

    if (nr < 0 || nr >= (int)rnss.size())
    {
        return;
    }

    this->rns = rnss[nr];
}

void GraspRrtWindow::selectEEF(int nr)
{
    this->eef.reset();

    if (!robot)
    {
        return;
    }

    selectPlanSet(nr);
    /*  std::vector< EndEffectorPtr > eefs = robot->getEndEffectors();
        if (nr<0 || nr>=(int)eefs.size())
            return;
        this->eef = eefs[nr];*/
}

void GraspRrtWindow::selectColModelRobA(int nr)
{
    colModelRobA.reset();

    if (!robot)
    {
        return;
    }

    std::vector< RobotNodeSetPtr > rnss = robot->getRobotNodeSets();

    if (nr < 0 || nr >= (int)rnss.size())
    {
        return;
    }

    this->colModelRobA = robot->getRobotNodeSet(rnss[nr]->getName());
}

void GraspRrtWindow::selectColModelRobB(int nr)
{
    colModelRobB.reset();

    if (!robot)
    {
        return;
    }

    std::vector< RobotNodeSetPtr > rnss = robot->getRobotNodeSets();

    if (nr < 0 || nr >= (int)rnss.size())
    {
        return;
    }

    this->colModelRobB = robot->getRobotNodeSet(rnss[nr]->getName());
}

void GraspRrtWindow::selectColModelEnv(int nr)
{
    colModelEnv.reset();

    if (!scene)
    {
        return;
    }

    std::vector< SceneObjectSetPtr > rnss = scene->getSceneObjectSets();

    if (nr < 0 || nr >= (int)rnss.size())
    {
        return;
    }

    this->colModelEnv = scene->getSceneObjectSet(rnss[nr]->getName());
}

void GraspRrtWindow::buildRRTVisu()
{
    rrtSep->removeAllChildren();

    if (!cspace || !robot)
    {
        return;
    }

    boost::shared_ptr<Saba::CoinRrtWorkspaceVisualization> w(new Saba::CoinRrtWorkspaceVisualization(robot, cspace, eef->getGCP()->getName()));

    if (UI.checkBoxShowRRT->isChecked())
    {
        if (tree)
        {
            w->setCustomColor(0.5f, 0.5f, 0.5f);
            w->colorizeTreeNodes(2, Saba::RrtWorkspaceVisualization::eRed);
            w->addTree(tree, Saba::RrtWorkspaceVisualization::eCustom);
        }
    }

    if (UI.checkBoxShowSolution->isChecked() && solution)
    {
        w->addCSpacePath(solution);
    }

    if (UI.checkBoxShowSolutionOpti->isChecked() && solutionOptimized)
    {
        w->addCSpacePath(solutionOptimized, Saba::CoinRrtWorkspaceVisualization::eGreen);
    }

    w->addConfiguration(startConfig, Saba::CoinRrtWorkspaceVisualization::eGreen, 3.0f);
    SoSeparator* sol = w->getCoinVisualization();
    rrtSep->addChild(sol);
}

void GraspRrtWindow::testInit()
{
    // setup collision detection
    if (!test_cspace)
    {

        CDManagerPtr cdm(new CDManager());

        if (colModelRobA)
        {
            cdm->addCollisionModel(colModelRobA);
        }

        if (colModelRobB)
        {
            cdm->addCollisionModel(colModelRobB);
        }

        if (colModelEnv)
        {
            cdm->addCollisionModel(colModelEnv);
        }

        test_cspace.reset(new Saba::CSpaceSampled(robot, cdm, rns, 500000));
    }

    float sampl = (float)UI.doubleSpinBoxCSpaceSampling->value();
    float samplDCD = (float)UI.doubleSpinBoxColChecking->value();
    test_cspace->setSamplingSize(sampl);
    test_cspace->setSamplingSizeDCD(samplDCD);
    float minGraspScore = (float)UI.doubleSpinBoxMinGraspScore->value();

    test_graspRrt.reset(new Saba::GraspRrt(test_cspace, eef, targetObject, graspQuality, colModelEnv, 0.1f, minGraspScore));

    test_graspRrt->setStart(startConfig);
    eef->getGCP()->showCoordinateSystem(true);
    test_graspRrt->init();
}

void GraspRrtWindow::testGraspPose()
{
    if (!robot || !rns || !eef || !graspQuality)
    {
        return;
    }

    if (!test_cspace || !test_graspRrt)
    {
        testInit();
    }

    // create taregt on objects surface
    Eigen::Matrix4f globalGrasp;
    Eigen::VectorXf c(rns->getSize());
    //cspace->getRandomConfig(c);
    //rns->setJointValues(c);
    rns->getJointValues(c);
    test_graspRrt->calculateGlobalGraspPose(c, globalGrasp);

    VirtualRobot::ObstaclePtr o = VirtualRobot::Obstacle::createBox(10, 10, 10);
    o->setGlobalPose(globalGrasp);
    o->showCoordinateSystem(true);
    graspsSep->removeAllChildren();
    graspsSep->addChild(VirtualRobot::CoinVisualizationFactory::getCoinVisualization(o, VirtualRobot::SceneObject::Full));

    // move towards object
    Eigen::Matrix4f p = eef->getGCP()->getGlobalPose();
    test_graspRrt->createWorkSpaceSamplingStep(p, globalGrasp, c);
    robot->setJointValues(rns, c);

    // test
    /*Eigen::Matrix4f p1;
    p1.setIdentity();
    Eigen::Matrix4f p2;
    p2.setIdentity();
    p1(0,3) = 50;
    p2(0,3) = 200;
    Eigen::Matrix4f deltaPose = p1.inverse() * p2;
    cout << deltaPose << endl;

    deltaPose = p2 * p1.inverse();
    cout << deltaPose << endl;

    p2 = deltaPose * p1;
    cout << p2 << endl;
    p2 = p1 * deltaPose;
    cout << p2 << endl;*/
}

void GraspRrtWindow::plan()
{
    if (!robot || !rns || !eef || !graspQuality)
    {
        return;
    }

    // setup collision detection
    CDManagerPtr cdm(new CDManager());

    if (colModelRobA)
    {
        cdm->addCollisionModel(colModelRobA);
    }

    if (colModelRobB)
    {
        cdm->addCollisionModel(colModelRobB);
    }

    if (colModelEnv)
    {
        cdm->addCollisionModel(colModelEnv);
    }

    cdm->addCollisionModel(targetObject);
    cspace.reset(new Saba::CSpaceSampled(robot, cdm, rns, 500000));
    float sampl = (float)UI.doubleSpinBoxCSpaceSampling->value();
    float samplDCD = (float)UI.doubleSpinBoxColChecking->value();
    cspace->setSamplingSize(sampl);
    cspace->setSamplingSizeDCD(samplDCD);
    float minGraspScore = (float)UI.doubleSpinBoxMinGraspScore->value();

    Saba::GraspRrtPtr graspRrt(new Saba::GraspRrt(cspace, eef, targetObject, graspQuality, colModelEnv, 0.1f, minGraspScore));

    graspRrt->setStart(startConfig);

    bool planOk = false;
    bool planOK = graspRrt->plan();

    if (planOK)
    {
        VR_INFO << " Planning succeeded " << endl;
        solution = graspRrt->getSolution();
        Saba::ShortcutProcessorPtr postProcessing(new Saba::ShortcutProcessor(solution, cspace, false));
        solutionOptimized = postProcessing->optimize(100);
        tree = graspRrt->getTree();
        std::vector<Saba::GraspRrt::GraspInfo, Eigen::aligned_allocator<Saba::GraspRrt::GraspInfo> > vStoreGraspInfo;
        graspRrt->getGraspInfoResult(vStoreGraspInfo);
        grasps.clear();

        for (size_t i = 0; i < vStoreGraspInfo.size(); i++)
        {
            cout << "processing grasp " << i << endl;
            VirtualRobot::GraspPtr g(new VirtualRobot::Grasp("GraspRrt Grasp", robot->getType(), eef->getName(), vStoreGraspInfo[i].handToObjectTransform, "GraspRrt", vStoreGraspInfo[i].graspScore));
            grasps.push_back(g);
        }
    }
    else
    {
        VR_INFO << " Planning failed" << endl;
    }

    sliderSolution(1000);

    buildVisu();
}

void GraspRrtWindow::colModel()
{
    buildVisu();
}
void GraspRrtWindow::solutionSelected()
{
    sliderSolution(UI.horizontalSliderPos->sliderPosition());
}
void GraspRrtWindow::sliderSolution(int pos)
{
    if (!solution)
    {
        return;
    }

    Saba::CSpacePathPtr s = solution;

    if (UI.radioButtonSolutionOpti->isChecked() && solutionOptimized)
    {
        s = solutionOptimized;
    }

    float p = (float)pos / 1000.0f;
    Eigen::VectorXf iPos;
    s->interpolate(p, iPos);
    robot->setJointValues(rns, iPos);
    redraw();
}

void GraspRrtWindow::redraw()
{
    viewer->scheduleRedraw();
    UI.frameViewer->update();
    viewer->scheduleRedraw();
    this->update();
    viewer->scheduleRedraw();
}


void GraspRrtWindow::selectPlanSet(int nr)
{
    if (nr == 0)
    {
        selectRNS(planSetA.rns);
        selectEEF(planSetA.eef);
        selectColModelRobA(planSetA.colModelRob1);
        selectColModelRobB(planSetA.colModelRob2);
        selectColModelEnv(planSetA.colModelEnv);
        selectStart(UI.comboBoxStart->currentIndex());
    }
    else
    {
        selectRNS(planSetB.rns);
        selectEEF(planSetB.eef);
        selectColModelRobA(planSetB.colModelRob1);
        selectColModelRobB(planSetB.colModelRob2);
        selectColModelEnv(planSetB.colModelEnv);
        selectStart(UI.comboBoxStart->currentIndex());
    }
}


void GraspRrtWindow::closeEEF()
{
    if (eef)
    {
        eef->closeActors(targetObject);
    }

    redraw();
}

void GraspRrtWindow::openEEF()
{
    if (eef)
    {
        eef->openActors();
    }

    redraw();
}

