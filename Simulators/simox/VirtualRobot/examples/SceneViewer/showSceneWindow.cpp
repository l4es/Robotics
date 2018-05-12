
#include "showSceneWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"

#include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

showSceneWindow::showSceneWindow(std::string& sSceneFile, Qt::WFlags flags)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    sceneFile = sSceneFile;
    sceneSep = new SoSeparator;
    sceneSep->ref();
    sceneVisuSep = new SoSeparator;
    sceneSep->addChild(sceneVisuSep);
    graspVisu = new SoSeparator;
    sceneSep->addChild(graspVisu);
    coordVisu = new SoSeparator;
    sceneSep->addChild(coordVisu);

    setupUI();

    loadScene();

    viewer->viewAll();
}


showSceneWindow::~showSceneWindow()
{
    sceneSep->unref();
}


void showSceneWindow::setupUI()
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
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectScene()));
    connect(UI.comboBoxRobot, SIGNAL(activated(int)), this, SLOT(selectRobot(int)));
    connect(UI.comboBoxObject, SIGNAL(activated(int)), this, SLOT(selectObject(int)));
    connect(UI.comboBoxRobotConfig, SIGNAL(activated(int)), this, SLOT(selectRobotConfig(int)));
    connect(UI.comboBoxTrajectory, SIGNAL(activated(int)), this, SLOT(selectTrajectory(int)));

    connect(UI.horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(sliderMoved(int)));

    connect(UI.pushButtonEEFClose, SIGNAL(clicked()), this, SLOT(closeHand()));
    connect(UI.pushButtonEEFOpen, SIGNAL(clicked()), this, SLOT(openHand()));
    connect(UI.comboBoxEEF, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));
    connect(UI.comboBoxGrasp, SIGNAL(activated(int)), this, SLOT(selectGrasp(int)));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxRoot, SIGNAL(clicked()), this, SLOT(showRoot()));

    /*
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
    connect(UI.checkBoxStructure, SIGNAL(clicked()), this, SLOT(robotStructure()));
    UI.checkBoxFullModel->setChecked(true);
    connect(UI.checkBoxFullModel, SIGNAL(clicked()), this, SLOT(robotFullModel()));
    connect(UI.checkBoxRobotCoordSystems, SIGNAL(clicked()), this, SLOT(robotCoordSystems()));
    connect(UI.comboBoxRobotNodeSet, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));
    connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
    connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));*/

}

QString showSceneWindow::formatString(const char* s, float f)
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


void showSceneWindow::resetSceneryAll()
{
    updateGui();
    buildVisu();
}



void showSceneWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

void showSceneWindow::colModel()
{
    buildVisu();
}


void showSceneWindow::showRoot()
{
    buildVisu();
}

void showSceneWindow::buildVisu()
{
    if (!scene)
    {
        return;
    }

    sceneVisuSep->removeAllChildren();
    SceneObject::VisualizationType visuType = SceneObject::Full;

    if (UI.checkBoxColModel->isChecked())
    {
        visuType = SceneObject::Collision;
    }

    visualization = scene->getVisualization<CoinVisualization>(visuType);
    SoNode* visualisationNode = NULL;

    if (visualization)
    {
        visualisationNode = visualization->getCoinVisualization();
    }

    if (visualisationNode)
    {
        sceneVisuSep->addChild(visualisationNode);
    }

    coordVisu->removeAllChildren();
    if (UI.checkBoxRoot->isChecked())
    {
        std::string rootText="ROOT";
        coordVisu->addChild(CoinVisualizationFactory::CreateCoordSystemVisualization(2.0f,&rootText));
    }

    updateGraspVisu();


}

void showSceneWindow::updateGraspVisu()
{
    // build grasp visu
    graspVisu->removeAllChildren();

    if (UI.comboBoxGrasp->currentIndex() > 0 && currentObject && currentEEF && currentGrasp)
    {
        //SoSeparator* visu = CoinVisualizationFactory::CreateGraspVisualization(currentGrasp, currentEEF,currentObject->getGlobalPose());
        Eigen::Matrix4f gp = currentGrasp->getTcpPoseGlobal(currentObject->getGlobalPose());
        SoMatrixTransform* mt = CoinVisualizationFactory::getMatrixTransformScaleMM2M(gp);
        graspVisu->addChild(mt);

        std::string t = currentGrasp->getName();
        SoSeparator* visu = CoinVisualizationFactory::CreateCoordSystemVisualization(1.0f, &t);

        if (visu)
        {
            graspVisu->addChild(visu);
        }
    }
}

int showSceneWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void showSceneWindow::quit()
{
    std::cout << "showSceneWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}


void showSceneWindow::sliderMoved(int pos)
{
    if (!currentTrajectory)
    {
        return;
    }

    float fpos = (float)pos / 999.0f;

    if (currentRobot)
    {
        currentRobot->setJointValues(currentTrajectory, fpos);
    }
}


void showSceneWindow::selectScene()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Scene File"), QString(), tr("XML Files (*.xml)"));
    sceneFile = std::string(fi.toAscii());
    loadScene();
}

void showSceneWindow::loadScene()
{
    sceneVisuSep->removeAllChildren();
    currentEEF.reset();
    currentGrasp.reset();
    currentGraspSet.reset();
    currentObject.reset();
    currentRobot.reset();
    currentTrajectory.reset();
    cout << "Loading Scene from " << sceneFile << endl;

    scene.reset();

    try
    {
        scene = SceneIO::loadScene(sceneFile);
    }
    catch (VirtualRobotException& e)
    {
        cout << "Could not find valid scene in file " << sceneFile << endl;
    }

    if (!scene)
    {
        // try manip object
        try
        {

            ManipulationObjectPtr mo = ObjectIO::loadManipulationObject(sceneFile);
            //mo = mo->clone(mo->getName());

            if (mo)
            {
                VR_INFO << "Loaded Manipulation object:" << endl;
                mo->print();
                scene.reset(new Scene(mo->getName()));
                scene->registerManipulationObject(mo);
            }
        }
        catch (VirtualRobotException& e)
        {
            cout << "Could not find valid manipulation object in file " << sceneFile << endl;
        }
    }

    if (!scene)
    {
        // try object
        try
        {

            ObstaclePtr mo = ObjectIO::loadObstacle(sceneFile);

            if (mo)
            {
                VR_INFO << "Loaded obstacle:" << endl;
                mo->print();
                scene.reset(new Scene(mo->getName()));
                scene->registerObstacle(mo);
            }
        }
        catch (VirtualRobotException& e)
        {
            cout << "Could not find valid obstacle in file " << sceneFile << endl;
        }
    }

    if (!scene)
    {
        cout << " ERROR while creating scene" << endl;
        return;
    }


    /*std::vector<VirtualRobot::ManipulationObjectPtr> mo;
    mo = scene->getManipulationObjects();
    cout << "Printing " << mo.size() << " objects" << endl;
    for (size_t i=0;i<mo.size();i++)
    {
        mo[i]->print();
        mo[i]->showCoordinateSystem(true);
        Eigen::Vector3f c = mo[i]->getCoMGlobal();
        cout << "com global: \n" << c << endl;
        c = mo[i]->getCoMLocal();
        cout << "com local: \n" << c << endl;
        //mo[i]->showBoundingBox(true);
    }*/
    /*std::vector<VirtualRobot::ObstaclePtr> o;
    o = scene->getObstacles();
    cout << "Printing " << o.size() << " obstacles" << endl;
    for (size_t i=0;i<o.size();i++)
    {
        o[i]->print();
        o[i]->showCoordinateSystem(true);
        Eigen::Vector3f c = o[i]->getCoMGlobal();
        cout << "com global: \n" << c << endl;
        c = o[i]->getCoMLocal();
        cout << "com local: \n" << c << endl;
        //mo[i]->showBoundingBox(true);
    }*/

    // get nodes
    /*m_pRobot->getRobotNodes(allRobotNodes);
    m_pRobot->getRobotNodeSets(robotNodeSets);
    m_pRobot->getEndEffectors(eefs);
    updateEEFBox();
    updateRNSBox();
    selectRNS(0);
    if (allRobotNodes.size()==0)
        selectJoint(-1);
    else
        selectJoint(0);



    displayTriangles();

    // build visualization
    collisionModel();
    robotStructure();*/
    updateGui();
    buildVisu();
    viewer->viewAll();
}

void showSceneWindow::selectRobot(int nr)
{
    UI.comboBoxRobotConfig->clear();
    UI.comboBoxTrajectory->clear();
    UI.comboBoxEEF->clear();
    currentRobot.reset();

    if (nr < 0 || nr >= UI.comboBoxRobot->count() || !scene)
    {
        return;
    }

    std::string robName(UI.comboBoxRobot->currentText().toAscii());
    currentRobot = scene->getRobot(robName);

    if (!currentRobot)
    {
        return;
    }

    std::vector<VirtualRobot::RobotConfigPtr> roc = scene->getRobotConfigs(currentRobot);

    for (size_t i = 0; i < roc.size(); i++)
    {
        QString rn = roc[i]->getName().c_str();
        UI.comboBoxRobotConfig->addItem(rn);
    }

    if (roc.size() > 0)
    {
        UI.comboBoxRobotConfig->setCurrentIndex(0);
    }

    std::vector<VirtualRobot::TrajectoryPtr> tr = scene->getTrajectories(currentRobot->getName());

    for (size_t i = 0; i < tr.size(); i++)
    {
        QString rn = tr[i]->getName().c_str();
        UI.comboBoxTrajectory->addItem(rn);
    }

    if (tr.size() > 0)
    {
        UI.comboBoxTrajectory->setCurrentIndex(0);
    }


    std::vector<VirtualRobot::EndEffectorPtr> eefs = currentRobot->getEndEffectors();

    for (size_t i = 0; i < eefs.size(); i++)
    {
        QString rn = eefs[i]->getName().c_str();
        UI.comboBoxEEF->addItem(rn);
    }

    selectEEF(0);


    selectRobotConfig(0);
    selectTrajectory(0);
}

void showSceneWindow::selectRobotConfig(int nr)
{
    if (nr < 0 || nr >= UI.comboBoxRobotConfig->count() || !scene || !currentRobot)
    {
        return;
    }

    std::string s(UI.comboBoxRobotConfig->currentText().toAscii());
    VirtualRobot::RobotConfigPtr rc = scene->getRobotConfig(currentRobot->getName(), s);

    if (!rc)
    {
        return;
    }

    currentRobot->setJointValues(rc);
}

void showSceneWindow::selectTrajectory(int nr)
{
    UI.horizontalSlider->setSliderPosition(0);

    if (nr < 0 || nr >= UI.comboBoxTrajectory->count() || !scene)
    {
        currentTrajectory.reset();
        UI.horizontalSlider->setEnabled(false);
        return;
    }

    UI.horizontalSlider->setEnabled(true);
    std::string s(UI.comboBoxTrajectory->currentText().toAscii());
    currentTrajectory = scene->getTrajectory(s);
    sliderMoved(0);
}

void showSceneWindow::selectEEF(int nr)
{
    if (nr < 0 || nr >= UI.comboBoxEEF->count() || !currentRobot)
    {
        currentEEF.reset();
        return;
    }

    std::string eefStr(UI.comboBoxEEF->currentText().toAscii());
    currentEEF = currentRobot->getEndEffector(eefStr);
    updateGrasps();
}

void showSceneWindow::selectObject(int nr)
{
    if (!scene || nr < 0 || nr >= UI.comboBoxObject->count())
    {
        return;
    }

    std::string ob(UI.comboBoxObject->currentText().toAscii());
    currentObject.reset();

    if (scene->hasManipulationObject(ob))
    {
        VirtualRobot::ManipulationObjectPtr mo = scene->getManipulationObject(ob);
        currentObject = boost::dynamic_pointer_cast<SceneObject>(mo);
    }

    updateGrasps();
}

void showSceneWindow::selectGrasp(int nr)
{
    currentGrasp.reset();

    if (nr <= 0 || nr >= UI.comboBoxGrasp->count() || !currentGraspSet)
    {
        return;
    }

    std::string grStr(UI.comboBoxGrasp->currentText().toAscii());

    if (currentGraspSet->hasGrasp(grStr))
    {
        currentGrasp = currentGraspSet->getGrasp(grStr);
    }

    updateGraspVisu();
}

void showSceneWindow::updateGui()
{
    UI.comboBoxObject->clear();
    UI.comboBoxRobot->clear();
    UI.comboBoxRobotConfig->clear();
    UI.comboBoxTrajectory->clear();
    UI.comboBoxEEF->clear();


    currentRobot.reset();

    if (!scene)
    {
        return;
    }

    std::vector<VirtualRobot::RobotPtr> robs = scene->getRobots();

    for (size_t i = 0; i < robs.size(); i++)
    {
        QString rn = robs[i]->getName().c_str();
        UI.comboBoxRobot->addItem(rn);
    }

    std::vector<VirtualRobot::ManipulationObjectPtr> mos = scene->getManipulationObjects();

    for (size_t i = 0; i < mos.size(); i++)
    {
        QString mn = mos[i]->getName().c_str();
        UI.comboBoxObject->addItem(mn);
    }

    std::vector<VirtualRobot::ObstaclePtr> obs = scene->getObstacles();

    for (size_t i = 0; i < obs.size(); i++)
    {
        QString on = obs[i]->getName().c_str();
        UI.comboBoxObject->addItem(on);
    }

    if (robs.size() > 0)
    {
        UI.comboBoxRobot->setCurrentIndex(0);
        selectRobot(0);
    }

    if (obs.size() > 0)
    {
        UI.comboBoxRobot->setCurrentIndex(0);
        selectObject(0);
    }
}

void showSceneWindow::updateGrasps()
{
    currentGraspSet.reset();
    UI.comboBoxGrasp->clear();
    QString t("-");
    UI.comboBoxGrasp->addItem(t);
    VirtualRobot::ManipulationObjectPtr mo = boost::dynamic_pointer_cast<ManipulationObject>(currentObject);

    if (mo && currentEEF)
    {
        currentGraspSet = mo->getGraspSet(currentEEF);

        if (currentGraspSet)
            for (unsigned int i = 0; i < currentGraspSet->getSize(); i++)
            {
                t = currentGraspSet->getGrasp(i)->getName().c_str();
                UI.comboBoxGrasp->addItem(t);
            }
    }

    UI.comboBoxGrasp->setCurrentIndex(0);
    selectGrasp(0);
}

void showSceneWindow::closeHand()
{
    if (!currentEEF)
    {
        return;
    }

    VirtualRobot::SceneObjectPtr so;

    if (UI.comboBoxObject->currentIndex() >= 0)
    {
        if (UI.comboBoxObject->currentIndex() < (int)scene->getManipulationObjects().size())
        {
            std::string s(UI.comboBoxObject->currentText().toAscii());
            so = scene->getManipulationObject(s);
        }
        else
        {
            std::string s(UI.comboBoxObject->currentText().toAscii());
            so = scene->getObstacle(s);
        }
    }

    currentEEF->closeActors(so);
}

void showSceneWindow::openHand()
{
    if (!currentEEF)
    {
        return;
    }

    currentEEF->openActors();
}

