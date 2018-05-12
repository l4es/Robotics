
#include "showRobotWindow.h"
#include <VirtualRobot/COLLADA/ColladaIO.h>
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include <VirtualRobot/RuntimeEnvironment.h>

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

showRobotWindow::showRobotWindow(std::string& sRobotFilename, Qt::WFlags flags)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;
    //this->setCaption(QString("ShowRobot - KIT - Humanoids Group"));
    //resize(1100, 768);

    useColModel = false;
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(sRobotFilename);
    m_sRobotFilename = sRobotFilename;
    sceneSep = new SoSeparator;
    sceneSep->ref();
    robotSep = new SoSeparator;

    /*SoShapeHints * shapeHints = new SoShapeHints;
    shapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
    shapeHints->shapeType = SoShapeHints::UNKNOWN_SHAPE_TYPE;
    sceneSep->addChild(shapeHints);*/
    /*SoLightModel * lightModel = new SoLightModel;
    lightModel->model = SoLightModel::BASE_COLOR;
    sceneSep->addChild(lightModel);*/

    sceneSep->addChild(robotSep);

    setupUI();

    loadRobot();

    viewer->viewAll();
}


showRobotWindow::~showRobotWindow()
{
    robot.reset();
    sceneSep->unref();
}

/*
void CShowRobotWindow::saveScreenshot()
{
    static int counter = 0;
    SbString framefile;

    framefile.sprintf("MPL_Render_Frame%06d.png", counter);
    counter++;

    viewer->getSceneManager()->render();
    viewer->getSceneManager()->scheduleRedraw();
    QGLWidget* w = (QGLWidget*)viewer->getGLWidget();

    QImage i = w->grabFrameBuffer();
    bool bRes = i.save(framefile.getString(), "PNG");
    if (bRes)
        cout << "wrote image " << counter << endl;
    else
        cout << "failed writing image " << counter << endl;

}*/

void showRobotWindow::setupUI()
{
    UI.setupUi(this);
    //centralWidget()->setLayout(UI.gridLayoutViewer);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(true);
#ifdef WIN32
    //#ifndef _DEBUG
    viewer->setAntialiasing(true, 4);
    //#endif
#endif
    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));
    connect(UI.pushButtonSave, SIGNAL(clicked()), this, SLOT(saveRobot()));

    /*connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeHand()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openHand()));
    connect(UI.comboBoxEndEffector, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));*/

    connect(UI.checkBoxPhysicsCoM, SIGNAL(clicked()), this, SLOT(displayPhysics()));
    connect(UI.checkBoxPhysicsInertia, SIGNAL(clicked()), this, SLOT(displayPhysics()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(rebuildVisualization()));
    connect(UI.checkBoxStructure, SIGNAL(clicked()), this, SLOT(robotStructure()));
    UI.checkBoxFullModel->setChecked(true);
    connect(UI.checkBoxFullModel, SIGNAL(clicked()), this, SLOT(robotFullModel()));
    connect(UI.checkBoxRobotCoordSystems, SIGNAL(clicked()), this, SLOT(robotCoordSystems()));
    connect(UI.checkBoxShowCoordSystem, SIGNAL(clicked()), this, SLOT(showCoordSystem()));
    connect(UI.checkBoxSensors, SIGNAL(clicked()), this, SLOT(showSensors()));
    connect(UI.comboBoxRobotNodeSet, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));
    connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
    connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));

}

QString showRobotWindow::formatString(const char* s, float f)
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


void showRobotWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    std::vector<float> jv(allRobotNodes.size(), 0.0f);
    robot->setJointValues(allRobotNodes, jv);

    selectJoint(UI.comboBoxJoint->currentIndex());
}



void showRobotWindow::displayTriangles()
{
    QString text1, text2, text3;
    int trisAllFull, trisRNSFull, trisJointFull;
    trisAllFull = trisRNSFull = trisJointFull = 0;
    int trisAllCol, trisRNSCol, trisJointCol;
    trisAllCol = trisRNSCol = trisJointCol = 0;

    if (robot)
    {
        trisAllFull = robot->getNumFaces(false);
        trisAllCol = robot->getNumFaces(true);
        trisRNSFull = trisAllFull;
        trisRNSCol = trisAllCol;
    }

    if (currentRobotNodeSet)
    {
        trisRNSFull = currentRobotNodeSet->getNumFaces(false);
        trisRNSCol = currentRobotNodeSet->getNumFaces(true);
    }

    if (currentRobotNode)
    {
        trisJointFull = currentRobotNode->getNumFaces(false);
        trisJointCol = currentRobotNode->getNumFaces(true);
    }

    if (UI.checkBoxColModel->checkState() == Qt::Checked)
    {
        text1 = tr("Total\t:") + QString::number(trisAllCol);
        text2 = tr("RobotNodeSet:\t") + QString::number(trisRNSCol);
        text3 = tr("Joint:\t") + QString::number(trisJointCol);
    }
    else
    {
        text1 = tr("Total:\t") + QString::number(trisAllFull);
        text2 = tr("RobotNodeSet:\t") + QString::number(trisRNSFull);
        text3 = tr("Joint:\t") + QString::number(trisJointFull);
    }

    UI.labelInfo1->setText(text1);
    UI.labelInfo2->setText(text2);
    UI.labelInfo3->setText(text3);
}

void showRobotWindow::robotFullModel()
{
    if (!robot)
    {
        return;
    }

    bool showFullModel = UI.checkBoxFullModel->checkState() == Qt::Checked;

    robot->setupVisualization(showFullModel, true);

}

void showRobotWindow::rebuildVisualization()
{
    if (!robot)
    {
        return;
    }

    robotSep->removeAllChildren();
    //setRobotModelShape(UI.checkBoxColModel->state() == QCheckBox::On);
    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    visualization = robot->getVisualization<CoinVisualization>(colModel);
    SoNode* visualisationNode = NULL;

    if (visualization)
    {
        visualisationNode = visualization->getCoinVisualization();
    }

    if (visualisationNode)
    {
        robotSep->addChild(visualisationNode);
    }

    selectJoint(UI.comboBoxJoint->currentIndex());

    UI.checkBoxStructure->setEnabled(!useColModel);
    UI.checkBoxFullModel->setEnabled(!useColModel);
    UI.checkBoxRobotCoordSystems->setEnabled(!useColModel);

}


void showRobotWindow::displayPhysics()
{
    if (!robot)
    {
        return;
    }

    physicsCoMEnabled = UI.checkBoxPhysicsCoM->checkState() == Qt::Checked;
    physicsInertiaEnabled = UI.checkBoxPhysicsInertia->checkState() == Qt::Checked;
    robot->showPhysicsInformation(physicsCoMEnabled, physicsInertiaEnabled);

    // rebuild visualization
    rebuildVisualization();

}
void showRobotWindow::showRobot()
{
    //m_pGraspScenery->showRobot(m_pShowRobot->state() == QCheckBox::On);
}

void showRobotWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}




int showRobotWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void showRobotWindow::quit()
{
    std::cout << "CShowRobotWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void showRobotWindow::updateJointBox()
{
    UI.comboBoxJoint->clear();

    for (unsigned int i = 0; i < currentRobotNodes.size(); i++)
    {
        UI.comboBoxJoint->addItem(QString(currentRobotNodes[i]->getName().c_str()));
    }
}

void showRobotWindow::updateRNSBox()
{
    UI.comboBoxRobotNodeSet->clear();
    UI.comboBoxRobotNodeSet->addItem(QString("<All>"));

    for (unsigned int i = 0; i < robotNodeSets.size(); i++)
    {
        UI.comboBoxRobotNodeSet->addItem(QString(robotNodeSets[i]->getName().c_str()));
    }
}

void showRobotWindow::selectRNS(int nr)
{
    currentRobotNodeSet.reset();
    cout << "Selecting RNS nr " << nr << endl;

    if (nr <= 0)
    {
        // all joints
        currentRobotNodes = allRobotNodes;
    }
    else
    {
        nr--;

        if (nr >= (int)robotNodeSets.size())
        {
            return;
        }

        currentRobotNodeSet = robotNodeSets[nr];
        currentRobotNodes = currentRobotNodeSet->getAllRobotNodes();
        /*cout << "HIGHLIGHTING rns " << currentRobotNodeSet->getName() << endl;
        if (visualization)
        {

            robot->highlight(visualization,false);
            currentRobotNodeSet->highlight(visualization,true);
        }*/

    }

    updateJointBox();
    selectJoint(0);
    displayTriangles();
}

void showRobotWindow::selectJoint(int nr)
{
    if (currentRobotNode)
    {
        currentRobotNode->showBoundingBox(false);
    }

    currentRobotNode.reset();
    cout << "Selecting Joint nr " << nr << endl;

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    currentRobotNode = currentRobotNodes[nr];
    currentRobotNode->showBoundingBox(true, true);
    currentRobotNode->print();
    std::cout << "Offset: "  << currentRobotNode->getJointValueOffset() << std::endl;
    float mi = currentRobotNode->getJointLimitLo();
    float ma = currentRobotNode->getJointLimitHi();
    QString qMin = QString::number(mi);
    QString qMax = QString::number(ma);
    UI.labelMinPos->setText(qMin);
    UI.labelMaxPos->setText(qMax);
    float j = currentRobotNode->getJointValue();
    UI.lcdNumberJointValue->display((double)j);

    if (fabs(ma - mi) > 0 && (currentRobotNode->isTranslationalJoint() || currentRobotNode->isRotationalJoint()))
    {
        UI.horizontalSliderPos->setEnabled(true);
        int pos = (int)((j - mi) / (ma - mi) * 1000.0f);
        UI.horizontalSliderPos->setValue(pos);
    }
    else
    {
        UI.horizontalSliderPos->setValue(500);
        UI.horizontalSliderPos->setEnabled(false);
    }

    if (currentRobotNodes[nr]->showCoordinateSystemState())
    {
        UI.checkBoxShowCoordSystem->setCheckState(Qt::Checked);
    }
    else
    {
        UI.checkBoxShowCoordSystem->setCheckState(Qt::Unchecked);
    }

    /*
        cout << "HIGHLIGHTING node " << currentRobotNodes[nr]->getName() << endl;

        if (visualization)
        {
            robot->highlight(visualization,false);
            currentRobotNode->highlight(visualization,true);
        }*/
    displayTriangles();
}

void showRobotWindow::jointValueChanged(int pos)
{
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    float fPos = currentRobotNodes[nr]->getJointLimitLo() + (float)pos / 1000.0f * (currentRobotNodes[nr]->getJointLimitHi() - currentRobotNodes[nr]->getJointLimitLo());
    robot->setJointValue(currentRobotNodes[nr], fPos);
    UI.lcdNumberJointValue->display((double)fPos);

#if 0
    RobotNodePtr rnl = robot->getRobotNode("LeftLeg_TCP");
    RobotNodePtr rnr = robot->getRobotNode("RightLeg_TCP");

    if (rnl && rnr)
    {
        cout << "LEFT:" << endl;
        MathTools::printMat(rnl->getGlobalPose());
        cout << "RIGHT:" << endl;
        MathTools::printMat(rnr->getGlobalPose());
    }

#endif
}

void showRobotWindow::showCoordSystem()
{
    float size = 0.75f;
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    // first check if robot node has a visualization
    /*VisualizationNodePtr visu = robotNodes[nr]->getVisualization();
    if (!visu)
    {
        // create dummy visu
        SoSeparator *s = new SoSeparator();
        VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
        robotNodes[nr]->setVisualization(visualizationNode);
        //visualizationNode->showCoordinateSystem(UI.checkBoxShowCoordSystem->checkState() == Qt::Checked, size);

    }*/

    currentRobotNodes[nr]->showCoordinateSystem(UI.checkBoxShowCoordSystem->checkState() == Qt::Checked, size);
    // rebuild visualization
    rebuildVisualization();
}


void showRobotWindow::showSensors()
{
    if (!robot)
    {
        return;
    }

    bool showSensors = UI.checkBoxSensors->isChecked();

    std::vector<SensorPtr> sensors = robot->getSensors();

    for (size_t i = 0; i < sensors.size(); i++)
    {
        sensors[i]->setupVisualization(showSensors, showSensors);
        sensors[i]->showCoordinateSystem(showSensors);
    }

    // rebuild visualization
    rebuildVisualization();
}



void showRobotWindow::selectRobot()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr("Collada Files (*.dae)"));
    std::string s = m_sRobotFilename = std::string(fi.toAscii());

    if (!s.empty())
    {
        m_sRobotFilename = s;
        loadRobot();
    }
}

void showRobotWindow::loadRobot()
{
    robotSep->removeAllChildren();
    cout << "Loading Robot from " << m_sRobotFilename << endl;
    currentEEF.reset();
    currentRobotNode.reset();
    currentRobotNodes.clear();
    currentRobotNodeSet.reset();
    robot.reset();

    try
    {
        robot = ColladaIO::loadRobot(m_sRobotFilename, 1000.0f);
        //robot = RobotIO::loadRobot(m_sRobotFilename,RobotIO::eFull);
    }
    catch (VirtualRobotException& e)
    {
        cout << " ERROR while creating robot" << endl;
        cout << e.what();
        return;
    }

    if (!robot)
    {
        cout << " ERROR while creating robot" << endl;
        return;
    }

    UI.checkBoxColModel->setChecked(false);
    UI.checkBoxFullModel->setChecked(true);
    UI.checkBoxPhysicsCoM->setChecked(false);
    UI.checkBoxPhysicsInertia->setChecked(false);
    UI.checkBoxRobotCoordSystems->setChecked(false);
    UI.checkBoxShowCoordSystem->setChecked(false);
    UI.checkBoxStructure->setChecked(false);

    // get nodes
    robot->getRobotNodes(allRobotNodes);
    robot->getRobotNodeSets(robotNodeSets);
    robot->getEndEffectors(eefs);
    //updateEEFBox();
    updateRNSBox();
    selectRNS(0);

    if (allRobotNodes.size() == 0)
    {
        selectJoint(-1);
    }
    else
    {
        selectJoint(0);
    }

    /*if (eefs.size()==0)
        selectEEF(-1);
    else
        selectEEF(0);*/

    displayTriangles();

    // build visualization
    rebuildVisualization();
    robotStructure();
    displayPhysics();
    viewer->viewAll();
}

void showRobotWindow::robotStructure()
{
    if (!robot)
    {
        return;
    }

    structureEnabled = UI.checkBoxStructure->checkState() == Qt::Checked;
    robot->showStructure(structureEnabled);
    // rebuild visualization
    rebuildVisualization();
}

void showRobotWindow::saveRobot()
{

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                       "",
                       tr("Simox XML (*.xml)"));

    if (fileName.isEmpty())
    {
        return;
    }

    std::string outFile = fileName.toLocal8Bit().constData();
    //std::string outFile("robot.xml");
    //store robot to file
    boost::filesystem::path relOutFile(outFile);
    boost::filesystem::path outFilename = relOutFile.filename();
    boost::filesystem::path relOutDir = relOutFile.parent_path();
    boost::filesystem::path absOutFile = boost::filesystem::absolute(relOutDir);
    std::string modelDir;
    modelDir = outFilename.stem().string();
    modelDir += "_models";
    //cout << "Rel Out File: " << relOutFile << endl;
    //cout << "Rel Out Dir: " << relOutDir << endl;
    cout << "Storing model to directory: " << absOutFile << endl;
    cout << "Model filename: " << outFilename << endl;
    cout << "IV Models directory: " << modelDir << endl;



    if (!boost::filesystem::is_directory(absOutFile))
    {
        cout << "Not a valid directory:" << absOutFile.string() << endl;
        return;
    }

    bool saveOK = false;

    try
    {
        saveOK = RobotIO::saveXML(robot, outFilename.string(), absOutFile.string(), modelDir);
    }
    catch (VirtualRobotException& e)
    {
        cout << " ERROR while creating robot" << endl;
        cout << e.what();
        return;
    }
}

void showRobotWindow::robotCoordSystems()
{
    if (!robot)
    {
        return;
    }

    bool robotAllCoordsEnabled = UI.checkBoxRobotCoordSystems->checkState() == Qt::Checked;
    robot->showCoordinateSystems(robotAllCoordsEnabled);
    // rebuild visualization
    rebuildVisualization();
}
/*
void showRobotWindow::closeHand()
{
    if (currentEEF)
        currentEEF->closeActors();
}

void showRobotWindow::openHand()
{
#if 0
    if (robot)
    {
        float randMult = (float)(1.0/(double)(RAND_MAX));
        std::vector<RobotNodePtr> rn = robot->getRobotNodes();
        std::vector<RobotNodePtr> rnJoints;
        for (size_t j=0;j<rn.size();j++)
        {
            if (rn[j]->isRotationalJoint())
                rnJoints.push_back(rn[j]);
        }
        int loops = 10000;
        clock_t startT = clock();
        for (int i=0;i<loops;i++)
        {
            std::vector<float> jv;
            for (size_t j=0;j<rnJoints.size();j++)
            {
                float t = (float)rand() * randMult; // value from 0 to 1
                t = rnJoints[j]->getJointLimitLo() + (rnJoints[j]->getJointLimitHi() - rnJoints[j]->getJointLimitLo())*t;
                jv.push_back(t);
            }
            robot->setJointValues(rnJoints,jv);
        }
        clock_t endT = clock();

        float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        cout << "RobotNodes:" << rn.size() << endl;
        cout << "Joints:" << rnJoints.size() << endl;
        cout << "loops:" << loops << ". time (ms):" << diffClock << ". Per loop:" << diffClock/(float)loops << endl;
    }
#endif
    if (currentEEF)
        currentEEF->openActors();
}

void showRobotWindow::selectEEF( int nr )
{
    cout << "Selecting EEF nr " << nr << endl;
    if (nr<0 || nr>=(int)eefs.size())
        return;
    currentEEF = eefs[nr];
}

void showRobotWindow::updateEEFBox()
{
    UI.comboBoxEndEffector->clear();

    for (unsigned int i=0;i<eefs.size();i++)
    {
        UI.comboBoxEndEffector->addItem(QString(eefs[i]->getName().c_str()));
    }
}*/
