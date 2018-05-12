
#include "showRobotWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"

#include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osgDB/WriteFile>
#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

showRobotWindow::showRobotWindow(std::string& sRobotFilename, Qt::WFlags flags)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;
    //this->setCaption(QString("ShowRobot - KIT - Humanoids Group"));
    resize(1100, 768);

    useColModel = false;
    m_sRobotFilename = sRobotFilename;

    osgRoot = new osg::Group();
    osgRobot = new osg::Group();
    osgRoot->addChild(osgRobot);
    /*sceneSep = new SoSeparator;
    sceneSep->ref();
    robotSep = new SoSeparator;*/

    //sceneSep->addChild(robotSep);

    setupUI();

    loadRobot();
    osgWidget->viewAll();
    //m_pExViewer->viewAll();
}


showRobotWindow::~showRobotWindow()
{
    //sceneSep->unref();
}

/*
void CShowRobotWindow::saveScreenshot()
{
    static int counter = 0;
    SbString framefile;

    framefile.sprintf("MPL_Render_Frame%06d.png", counter);
    counter++;

     //m_pExViewer->getSceneManager()->render();
     //m_pExViewer->getSceneManager()->scheduleRedraw();
    QGLWidget* w = (QGLWidget*) //m_pExViewer->getGLWidget();

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
    this->show();

    /*osg::Box *box = new osg::Box(osg::Vec3(0,0,0),1.0f);
    osg::ShapeDrawable* unitCubeDrawable = new osg::ShapeDrawable(box);

    // Declare a instance of the geode class:
    osg::Geode* basicShapesGeode = new osg::Geode();

    // Add the unit cube drawable to the geode:
    basicShapesGeode->addDrawable(unitCubeDrawable);
    osgRoot->addChild(basicShapesGeode);*/

    osgWidget = new VirtualRobot::osgViewerWidget(osgRoot, UI.frameViewer);
    osgWidget->show();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));

    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeHand()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openHand()));
    connect(UI.comboBoxEndEffector, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));


    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
    connect(UI.checkBoxStructure, SIGNAL(clicked()), this, SLOT(robotStructure()));
    UI.checkBoxFullModel->setChecked(true);
    connect(UI.checkBoxFullModel, SIGNAL(clicked()), this, SLOT(robotFullModel()));
    connect(UI.checkBoxRobotCoordSystems, SIGNAL(clicked()), this, SLOT(robotCoordSystems()));
    connect(UI.checkBoxShowCoordSystem, SIGNAL(clicked()), this, SLOT(showCoordSystem()));
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

void showRobotWindow::collisionModel()
{
    if (!robot)
    {
        return;
    }

    cout << "1:" << osgRobot->getNumChildren() << endl;

    if (osgRobot->getNumChildren() > 0)
    {
        osgRobot->removeChildren(0, osgRobot->getNumChildren());
    }

    cout << "2:" << osgRobot->getNumChildren() << endl;

    //setRobotModelShape(UI.checkBoxColModel->state() == QCheckBox::On);
    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    visualization = robot->getVisualization<OSGVisualization>(colModel);
    osg::Node* visualisationNode = NULL;

    if (visualization)
    {
        visualisationNode = visualization->getOSGVisualization();
    }

    if (visualisationNode)
    {
        osgRobot->addChild(visualisationNode);
    }

    selectJoint(UI.comboBoxJoint->currentIndex());

    UI.checkBoxStructure->setEnabled(!useColModel);
    UI.checkBoxFullModel->setEnabled(!useColModel);
    UI.checkBoxRobotCoordSystems->setEnabled(!useColModel);

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
    /*SoQt::show(this);
    SoQt::mainLoop();*/
    return 0;
}


void showRobotWindow::quit()
{
    std::cout << "CShowRobotWindow: Closing" << std::endl;
    this->close();
    //SoQt::exitMainLoop();
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
        cout << "HIGHLIGHTING rns " << currentRobotNodeSet->getName() << endl;
        /*if (visualization)
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
    currentRobotNode.reset();
    cout << "Selecting Joint nr " << nr << endl;

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    currentRobotNode = currentRobotNodes[nr];
    currentRobotNode->print();
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

    /*cout << "HIGHLIGHTING node " << currentRobotNodes[nr]->getName() << endl;

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
    collisionModel();
}



void showRobotWindow::selectRobot()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr("XML Files (*.xml)"));
    m_sRobotFilename = std::string(fi.toAscii());
    loadRobot();
}

void showRobotWindow::loadRobot()
{
    if (osgRobot->getNumChildren() > 0)
    {
        osgRobot->removeChildren(0, osgRobot->getNumChildren());
    }

    cout << "Loading Robot from " << m_sRobotFilename << endl;

    try
    {
        robot = RobotIO::loadRobot(m_sRobotFilename, RobotIO::eFull);
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

    // get nodes
    robot->getRobotNodes(allRobotNodes);
    robot->getRobotNodeSets(robotNodeSets);
    robot->getEndEffectors(eefs);
    updateEEFBox();
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

    if (eefs.size() == 0)
    {
        selectEEF(-1);
    }
    else
    {
        selectEEF(0);
    }

    displayTriangles();

    // build visualization
    collisionModel();
    robotStructure();


    /*boost::shared_ptr<VirtualRobot::OSGVisualization> visualizationCol = robot->getVisualization<OSGVisualization>(SceneObject::Collision);
    osg::Node* visualisationNodeCol = NULL;
    if (visualizationCol)
        visualisationNodeCol = visualizationCol->getOSGVisualization();
    osgDB::writeNodeFile(*visualisationNodeCol, std::string(SIMOX_BASE_DIR "/colModel.osg"));

    boost::shared_ptr<VirtualRobot::OSGVisualization> visualizationFul = robot->getVisualization<OSGVisualization>(SceneObject::Collision);
    osg::Node* visualisationNodeFul = NULL;
    if (visualizationFul)
        visualisationNodeFul = visualizationFul->getOSGVisualization();
    osgDB::writeNodeFile(*visualisationNodeFul, std::string(SIMOX_BASE_DIR "/fullModel.osg"));*/
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
    collisionModel();
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
    collisionModel();
}

void showRobotWindow::closeHand()
{
    if (currentEEF)
    {
        currentEEF->closeActors();
    }
}

void showRobotWindow::openHand()
{
    if (currentEEF)
    {
        currentEEF->openActors();
    }
}

void showRobotWindow::selectEEF(int nr)
{
    cout << "Selecting EEF nr " << nr << endl;

    if (nr < 0 || nr >= (int)eefs.size())
    {
        return;
    }

    currentEEF = eefs[nr];
}

void showRobotWindow::updateEEFBox()
{
    UI.comboBoxEndEffector->clear();

    for (unsigned int i = 0; i < eefs.size(); i++)
    {
        UI.comboBoxEndEffector->addItem(QString(eefs[i]->getName().c_str()));
    }
}
