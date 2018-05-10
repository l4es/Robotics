
#include "IKRRTWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/RobotIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "MotionPlanning/Planner/GraspIkRrt.h"
#include "MotionPlanning/CSpace/CSpaceSampled.h"
#include "MotionPlanning/PostProcessing/ShortcutProcessor.h"
#include <MotionPlanning/Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h>
#include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <QImage>
#include <QGLWidget>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoMatrixTransform.h>

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

IKRRTWindow::IKRRTWindow(std::string& sceneFile, std::string& reachFile, std::string& rns, std::string& eef, std::string& colModel, std::string& colModelRob, Qt::WFlags flags)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    this->sceneFile = sceneFile;
    this->reachFile = reachFile;
    eefName = eef;
    rnsName = rns;
    this->colModelName = colModel;
    this->colModelNameRob = colModelRob;

    sceneSep = new SoSeparator;
    sceneSep->ref();
    robotSep = new SoSeparator;
    objectSep = new SoSeparator;
    graspsSep = new SoSeparator;
    reachableGraspsSep = new SoSeparator;
    reachabilitySep = new SoSeparator;
    obstaclesSep = new SoSeparator;
    rrtSep = new SoSeparator;

    playbackMode = false;

    //sceneSep->addChild(robotSep);

    sceneSep->addChild(robotSep);
    sceneSep->addChild(objectSep);
    sceneSep->addChild(graspsSep);
    sceneSep->addChild(reachableGraspsSep);
    sceneSep->addChild(reachabilitySep);
    sceneSep->addChild(obstaclesSep);
    sceneSep->addChild(rrtSep);

    setupUI();

    loadScene();

    loadReach();

    viewer->viewAll();

    SoSensorManager* sensor_mgr = SoDB::getSensorManager();
    SoTimerSensor* timer = new SoTimerSensor(timerCB, this);
    timer->setInterval(SbTime(TIMER_MS / 1000.0f));
    sensor_mgr->insertTimerSensor(timer);
}


IKRRTWindow::~IKRRTWindow()
{
    sceneSep->unref();
}


void IKRRTWindow::timerCB(void* data, SoSensor* sensor)
{
    IKRRTWindow* ikWindow = static_cast<IKRRTWindow*>(data);
    float x[6];
    x[0] = (float)ikWindow->UI.horizontalSliderX->value();
    x[1] = (float)ikWindow->UI.horizontalSliderY->value();
    x[2] = (float)ikWindow->UI.horizontalSliderZ->value();
    x[3] = (float)ikWindow->UI.horizontalSliderRo->value();
    x[4] = (float)ikWindow->UI.horizontalSliderPi->value();
    x[5] = (float)ikWindow->UI.horizontalSliderYa->value();
    x[0] /= 10.0f;
    x[1] /= 10.0f;
    x[2] /= 10.0f;
    x[3] /= 300.0f;
    x[4] /= 300.0f;
    x[5] /= 300.0f;

    if (x[0] != 0 || x[1] != 0 || x[2] != 0 || x[3] != 0 || x[4] != 0 || x[5] != 0)
    {
        ikWindow->updateObject(x);
        ikWindow->redraw();
    }

    int maxSlider = 200;

    if (ikWindow->playbackMode && ikWindow->playCounter <= maxSlider)
    {
        if (ikWindow->playCounter == 0)
        {
            ikWindow->openEEF();
            ikWindow->sliderSolution(0);
            ikWindow->playCounter++;
        }
        else if (ikWindow->playCounter == maxSlider)
        {
            ikWindow->sliderSolution(1000);
            ikWindow->closeEEF();
            cout << "Stopping playback" << endl;
            ikWindow->playbackMode = false;
        }
        else
        {
            ikWindow->playCounter++;
            float pos = (float)ikWindow->playCounter / (float)maxSlider;
            ikWindow->sliderSolution((int)(pos * 1000.0f));
        }

        ikWindow->redraw();
        ikWindow->saveScreenshot();
    }
}


void IKRRTWindow::setupUI()
{
    UI.setupUi(this);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(true);
#ifdef WIN32
    //#ifndef _DEBUG
    viewer->setAntialiasing(true, 8);
    //#endif
#endif
    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer->setFeedbackVisibility(false);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
    connect(UI.pushButtonIK, SIGNAL(clicked()), this, SLOT(searchIK()));

    connect(UI.checkBoxSolution, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxTCP, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxGraspSet, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxReachableGrasps, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxReachabilitySpace, SIGNAL(clicked()), this, SLOT(reachVisu()));
    connect(UI.pushButtonIKRRT, SIGNAL(clicked()), this, SLOT(planIKRRT()));
    connect(UI.pushButtonPlay, SIGNAL(clicked()), this, SLOT(playAndSave()));

    connect(UI.horizontalSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectX()));
    connect(UI.horizontalSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectY()));
    connect(UI.horizontalSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectZ()));
    connect(UI.horizontalSliderRo, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectA()));
    connect(UI.horizontalSliderPi, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectB()));
    connect(UI.horizontalSliderYa, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectG()));
    connect(UI.horizontalSliderSolution, SIGNAL(valueChanged(int)), this, SLOT(sliderSolution(int)));

    UI.checkBoxColCheckIK->setChecked(false);
    UI.checkBoxReachabilitySpaceIK->setChecked(false);

}

void IKRRTWindow::playAndSave()
{
    if (playbackMode)
    {
        playbackMode = false;
    }
    else
    {
        playCounter = 0;
        playbackMode = true;
    }
}

QString IKRRTWindow::formatString(const char* s, float f)
{
    QString str1(s);

    if (f >= 0)
    {
        str1 += " ";
    }

    if (fabs(f) < 1000)
    {
        str1 += " ";
    }

    if (fabs(f) < 100)
    {
        str1 += " ";
    }

    if (fabs(f) < 10)
    {
        str1 += " ";
    }

    QString str1n;
    str1n.setNum(f, 'f', 3);
    str1 = str1 + str1n;
    return str1;
}


void IKRRTWindow::resetSceneryAll()
{
    if (rns && robot)
    {
        robot->setJointValues(rns, startConfig);
    }
}


void IKRRTWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}



void IKRRTWindow::saveScreenshot()
{
    static int counter = 0;
    SbString framefile;

    framefile.sprintf("renderFrame_%06d.png", counter);
    counter++;
    redraw();
    viewer->getSceneManager()->render();
    viewer->getSceneManager()->scheduleRedraw();
    QGLWidget* w = (QGLWidget*)viewer->getGLWidget();

    QImage i = w->grabFrameBuffer();
    bool bRes = i.save(framefile.getString(), "BMP");

    if (bRes)
    {
        cout << "wrote image " << counter << endl;
    }
    else
    {
        cout << "failed writing image " << counter << endl;
    }

}

void IKRRTWindow::buildVisu()
{
    showCoordSystem();

    robotSep->removeAllChildren();
    //bool colModel = (UI.checkBoxColModel->isChecked());
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    if (robot)
    {
        visualizationRobot = robot->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = visualizationRobot->getCoinVisualization();

        if (visualisationNode)
        {
            robotSep->addChild(visualisationNode);
            //visualizationRobot->highlight(true);
        }
    }

    objectSep->removeAllChildren();

    if (object)
    {
        SceneObject::VisualizationType colModel2 = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(object, colModel2);

        if (visualisationNode)
        {
            objectSep->addChild(visualisationNode);
        }
    }

    obstaclesSep->removeAllChildren();

    if (obstacles.size() > 0)
    {
        for (size_t i = 0; i < obstacles.size(); i++)
        {
            SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(obstacles[i], colModel);

            if (visualisationNode)
            {
                obstaclesSep->addChild(visualisationNode);
            }
        }
    }

    buildGraspSetVisu();

    buildRRTVisu();

    redraw();
}

int IKRRTWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void IKRRTWindow::quit()
{
    std::cout << "IKRRTWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void IKRRTWindow::loadScene()
{
    graspSet.reset();
    robot.reset();
    object.reset();
    obstacles.clear();
    eef.reset();
    ScenePtr scene = SceneIO::loadScene(sceneFile);

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


    std::vector< ManipulationObjectPtr > objects = scene->getManipulationObjects();

    if (objects.size() < 1)
    {
        VR_ERROR << "Need at least 1 object" << endl;
        return;
    }

    object = objects[0];
    VR_INFO << "using first manipulation object: " << object->getName() << endl;


    obstacles = scene->getObstacles();

    if (robot && object)
    {
        eef = robot->getEndEffector(eefName);

        if (!eef)
        {
            VR_ERROR << "Need a correct EEF in robot" << endl;
            return;
        }

        graspSet = object->getGraspSet(eef);

        rns = robot->getRobotNodeSet(rnsName);

        if (!rns)
        {
            VR_ERROR << "Need a correct RNS in robot" << endl;
        }

    }

    if (rns)
    {
        rns->getJointValues(startConfig);
    }

    buildVisu();

}


void IKRRTWindow::closeEEF()
{
    if (eef)
    {
        eef->closeActors(object);
    }

    redraw();

}

void IKRRTWindow::openEEF()
{
    if (eef)
    {
        eef->openActors();
    }

    redraw();

}



void IKRRTWindow::updateObject(float x[6])
{
    if (object)
    {
        //cout << "getGlobalPose robot:" << endl << robotEEF->getGlobalPose() << endl;
        //cout << "getGlobalPose TCP:" << endl <<  robotEEF_EEF->getTcp()->getGlobalPose() << endl;
        Eigen::Matrix4f m;
        MathTools::posrpy2eigen4f(x, m);

        m = object->getGlobalPose() * m;
        object->setGlobalPose(m);
        cout << "object " << endl;
        cout << m << endl;

    }

    redraw();

}

void IKRRTWindow::sliderReleased_ObjectX()
{
    UI.horizontalSliderX->setValue(0);
    buildVisu();
}

void IKRRTWindow::sliderReleased_ObjectY()
{
    UI.horizontalSliderY->setValue(0);
    buildVisu();
}

void IKRRTWindow::sliderReleased_ObjectZ()
{
    UI.horizontalSliderZ->setValue(0);
    buildVisu();
}

void IKRRTWindow::sliderReleased_ObjectA()
{
    UI.horizontalSliderRo->setValue(0);
    buildVisu();
}

void IKRRTWindow::sliderReleased_ObjectB()
{
    UI.horizontalSliderPi->setValue(0);
    buildVisu();
}

void IKRRTWindow::sliderReleased_ObjectG()
{
    UI.horizontalSliderYa->setValue(0);
    buildVisu();
}

void IKRRTWindow::showCoordSystem()
{
    if (eef)
    {
        RobotNodePtr tcp = eef->getTcp();

        if (!tcp)
        {
            return;
        }

        tcp->showCoordinateSystem(UI.checkBoxTCP->isChecked());
    }

    if (object)
    {
        object->showCoordinateSystem(UI.checkBoxTCP->isChecked());
    }
}

void IKRRTWindow::buildRRTVisu()
{
    rrtSep->removeAllChildren();

    if (!UI.checkBoxSolution->isChecked())
    {
        return;
    }

    if (!solution)
    {
        return;
    }

    boost::shared_ptr<Saba::CoinRrtWorkspaceVisualization> w(new Saba::CoinRrtWorkspaceVisualization(robot, cspace, eef->getTcpName()));

    if (tree)
    {
        w->addTree(tree);
    }

    if (tree2)
    {
        w->addTree(tree2);
    }

    //w->addCSpacePath(solution);
    if (solutionOptimized)
    {
        w->addCSpacePath(solutionOptimized, Saba::CoinRrtWorkspaceVisualization::eGreen);
    }

    //w->addConfiguration(startConfig,Saba::CoinRrtWorkspaceVisualization::eGreen,3.0f);
    SoSeparator* sol = w->getCoinVisualization();
    rrtSep->addChild(sol);
}

void IKRRTWindow::buildGraspSetVisu()
{
    graspsSep->removeAllChildren();

    if (UI.checkBoxGraspSet->isChecked() && eef && graspSet && object)
    {
        SoSeparator* visu = CoinVisualizationFactory::CreateGraspSetVisualization(graspSet, eef, object->getGlobalPose());

        if (visu)
        {
            graspsSep->addChild(visu);
        }
    }

    // show reachable graps
    reachableGraspsSep->removeAllChildren();

    if (UI.checkBoxReachableGrasps->isChecked() && eef && graspSet && object && reachSpace)
    {
        GraspSetPtr rg = reachSpace->getReachableGrasps(graspSet, object);

        if (rg->getSize() > 0)
        {
            SoSeparator* visu = CoinVisualizationFactory::CreateGraspSetVisualization(rg, eef, object->getGlobalPose());

            if (visu)
            {
                reachableGraspsSep->addChild(visu);
            }
        }
    }
}


void IKRRTWindow::reachVisu()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    reachabilitySep->removeAllChildren();

    if (UI.checkBoxReachabilitySpace->checkState() == Qt::Checked)
    {
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(reachSpace, VirtualRobot::ColorMap::eRed, true);

        if (visualisationNode)
        {
            reachabilitySep->addChild(visualisationNode);
        }

        /*
            boost::shared_ptr<VirtualRobot::CoinVisualization> visualization = reachSpace->getVisualization<CoinVisualization>();
            SoNode* visualisationNode = NULL;
            if (visualization)
                visualisationNode = visualization->getCoinVisualization();

            if (visualisationNode)
                reachabilitySep->addChild(visualisationNode);
        */
    }
}

void IKRRTWindow::loadReach()
{
    reachabilitySep->removeAllChildren();

    if (!robot)
    {
        return;
    }

    cout << "Loading Reachability from " << reachFile << endl;
    reachSpace.reset(new Reachability(robot));

    try
    {
        reachSpace->load(reachFile);
    }
    catch (VirtualRobotException& e)
    {
        cout << " ERROR while loading reach space" << endl;
        cout << e.what();
        reachSpace.reset();
        return;
    }

    reachSpace->print();

    buildVisu();
}
void IKRRTWindow::planIKRRT()
{

    GenericIKSolverPtr ikSolver(new GenericIKSolver(rns));

    if (UI.checkBoxReachabilitySpaceIK->checkState() == Qt::Checked)
    {
        ikSolver->setReachabilityCheck(reachSpace);
    }

    // setup collision detection
    CDManagerPtr cdm;

    if (UI.checkBoxColCheckIK->checkState() == Qt::Checked)
    {
        SceneObjectSetPtr colModelSet = robot->getRobotNodeSet(colModelName);
        SceneObjectSetPtr colModelSet2;

        if (!colModelNameRob.empty())
        {
            colModelSet2 = robot->getRobotNodeSet(colModelNameRob);
        }

        if (colModelSet)
        {
            cdm.reset(new CDManager());
            cdm->addCollisionModel(object);
            cdm->addCollisionModel(colModelSet);

            if (colModelSet2)
            {
                cdm->addCollisionModel(colModelSet2);
            }

            ikSolver->collisionDetection(cdm);
        }
    }
    else
    {
        cdm.reset(new CDManager());
    }

    ikSolver->setMaximumError(10.0f, 0.08f);
    ikSolver->setupJacobian(0.9f, 20);

    cspace.reset(new Saba::CSpaceSampled(robot, cdm, rns));

    GraspSetPtr graspSet = object->getGraspSet(robot->getType(), eefName);
    Saba::GraspIkRrtPtr ikRrt(new Saba::GraspIkRrt(cspace, object, ikSolver, graspSet));


    ikRrt->setStart(startConfig);
    bool planOK = ikRrt->plan();

    if (planOK)
    {
        VR_INFO << " Planning succeeded " << endl;
        solution = ikRrt->getSolution();
        Saba::ShortcutProcessorPtr postProcessing(new Saba::ShortcutProcessor(solution, cspace, false));
        solutionOptimized = postProcessing->optimize(100);
        tree = ikRrt->getTree();
        tree2 = ikRrt->getTree2();

    }
    else
    {
        VR_INFO << " Planning failed" << endl;
    }

    sliderSolution(1000);

    buildVisu();
}

void IKRRTWindow::colModel()
{
#if 0

    if (reachSpace && graspSet && object)
    {
        Eigen::Matrix4f m = reachSpace->sampleReachablePose();
        cout << "getEntry: " << (int)reachSpace->getEntry(m) << endl;
        /*SoSeparator* sep1 = new SoSeparator;
        SoSeparator *cs = CoinVisualizationFactory::CreateCoordSystemVisualization();
        SoMatrixTransform *mt = new SoMatrixTransform;
        SbMatrix ma(reinterpret_cast<SbMat*>(m.data()));
        mt->matrix.setValue(ma);
        sep1->addChild(mt);
        sep1->addChild(cs);
        sceneSep->addChild(sep1);*/


        GraspPtr g = graspSet->getGrasp(0);
        m = g->getObjectTargetPoseGlobal(m);
        object->setGlobalPose(m);
    }

#endif
    buildVisu();
}

void IKRRTWindow::searchIK()
{
    GenericIKSolverPtr ikSolver(new GenericIKSolver(rns));

    if (UI.checkBoxReachabilitySpaceIK->checkState() == Qt::Checked)
    {
        ikSolver->setReachabilityCheck(reachSpace);
    }

    // setup collision detection
    if (UI.checkBoxColCheckIK->checkState() == Qt::Checked)
    {
        SceneObjectSetPtr colModelSet = robot->getRobotNodeSet(colModelName);

        if (colModelSet)
        {
            CDManagerPtr cdm(new CDManager());
            cdm->addCollisionModel(object);
            cdm->addCollisionModel(colModelSet);
            ikSolver->collisionDetection(cdm);
        }
    }

    ikSolver->setMaximumError(5.0f, 0.05f);
    ikSolver->setupJacobian(0.9f, 20);
    GraspPtr grasp = ikSolver->solve(object, IKSolver::All, 10);

    if (grasp)
    {
        VR_INFO << "IK successful..." << endl;
    }
    else
    {
        VR_INFO << "IK failed..." << endl;
    }

    redraw();
}

void IKRRTWindow::sliderSolution(int pos)
{
    if (!solution)
    {
        return;
    }

    Saba::CSpacePathPtr s = solution;

    if (solutionOptimized)
    {
        s = solutionOptimized;
    }

    float p = (float)pos / 1000.0f;
    Eigen::VectorXf iPos;
    s->interpolate(p, iPos);
    robot->setJointValues(rns, iPos);

    redraw();
    //saveScreenshot();
}

void IKRRTWindow::redraw()
{
    viewer->scheduleRedraw();
    UI.frameViewer->update();
    viewer->scheduleRedraw();
    this->update();
    viewer->scheduleRedraw();
}


