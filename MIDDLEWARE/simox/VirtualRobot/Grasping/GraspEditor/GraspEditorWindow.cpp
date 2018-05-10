
#include "GraspEditorWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/RobotIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "VirtualRobot/SphereApproximator.h"
#include "VirtualRobot/Visualization/TriMeshModel.h"

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

#include <sstream>

#include <VirtualRobot/ui_GraspEditor.h>


using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

namespace VirtualRobot
{

    GraspEditorWindow::GraspEditorWindow(std::string& objFile, std::string& robotFile,
        bool embeddedGraspEditor, Qt::WFlags flags)
        : QMainWindow(NULL), UI(new Ui::MainWindowGraspEditor)
    {
        VR_INFO << " start " << endl;

        // Indicates whether this program is started inside another extern program
        this->embeddedGraspEditor = embeddedGraspEditor;

        objectFile = objFile;
        this->robotFile = robotFile;

        sceneSep = new SoSeparator;
        sceneSep->ref();
        robotSep = new SoSeparator;
        objectSep = new SoSeparator;
        eefVisu = new SoSeparator;
        graspSetVisu = new SoSeparator;

        //sceneSep->addChild(robotSep);

        sceneSep->addChild(eefVisu);
        sceneSep->addChild(objectSep);
        sceneSep->addChild(graspSetVisu);

#if 0
        // 2d map test
        Eigen::MatrixXf d(10, 10);

        for (int x = 0; x < 10; x++)
        for (int y = 0; y < 10; y++)
        {
            d(x, y) = (float)(x + y) / 20.0f;
        }

        SoSeparator* sep1 = CoinVisualizationFactory::Create2DMap(d, 10.0f, 10.0f, VirtualRobot::ColorMap::ColorMap(VirtualRobot::ColorMap::eHot), true);
        SoSeparator* sep2 = CoinVisualizationFactory::Create2DHeightMap(d, 10.0f, 10.0f, 50.0f);
        sceneSep->addChild(sep1);
        sceneSep->addChild(sep2);
#endif

#if 0
        SphereApproximatorPtr sa(new SphereApproximator());
        SphereApproximator::SphereApproximation app;
        sa->generateGraph(app, SphereApproximator::eIcosahedron, 3, 200.0f);
        cout << "nr faces:" << app.faces.size() << ", vert:" << app.vertices.size() << endl;

        TriMeshModelPtr tri = sa->generateTriMesh(app);
        cout << "2 nr faces:" << tri->faces.size() << ", vert:" << tri->vertices.size() << endl;
        SoNode* m = CoinVisualizationFactory::getCoinVisualization(tri, true);
        sceneSep->addChild(m);

#endif
        setupUI();

        loadObject();
        loadRobot();

        m_pExViewer->viewAll();

        SoSensorManager* sensor_mgr = SoDB::getSensorManager();
        timer = new SoTimerSensor(timerCB, this);
        timer->setInterval(SbTime(TIMER_MS / 1000.0f));
        sensor_mgr->insertTimerSensor(timer);
    }


    GraspEditorWindow::~GraspEditorWindow()
    {
        timer->unschedule();
        delete m_pExViewer;
        delete UI;
        sceneSep->unref();
    }


    void GraspEditorWindow::timerCB(void* data, SoSensor* sensor)
    {
        GraspEditorWindow* ikWindow = static_cast<GraspEditorWindow*>(data);
        float x[6];
        x[0] = (float)ikWindow->UI->horizontalSliderX->value();
        x[1] = (float)ikWindow->UI->horizontalSliderY->value();
        x[2] = (float)ikWindow->UI->horizontalSliderZ->value();
        x[3] = (float)ikWindow->UI->horizontalSliderRo->value();
        x[4] = (float)ikWindow->UI->horizontalSliderPi->value();
        x[5] = (float)ikWindow->UI->horizontalSliderYa->value();
        x[0] /= 10.0f;
        x[1] /= 10.0f;
        x[2] /= 10.0f;
        x[3] /= 300.0f;
        x[4] /= 300.0f;
        x[5] /= 300.0f;

        if (x[0] != 0 || x[1] != 0 || x[2] != 0 || x[3] != 0 || x[4] != 0 || x[5] != 0)
        {
            ikWindow->updateEEF(x);
        }
    }


    void GraspEditorWindow::setupUI()
    {
        UI->setupUi(this);
        m_pExViewer = new SoQtExaminerViewer(UI->frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

        // setup
        m_pExViewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
        m_pExViewer->setAccumulationBuffer(true);
#ifdef WIN32
#ifndef _DEBUG
        m_pExViewer->setAntialiasing(true, 4);
#endif
#endif
        m_pExViewer->setGLRenderAction(new SoLineHighlightRenderAction);
        m_pExViewer->setTransparencyType(SoGLRenderAction::BLEND);
        m_pExViewer->setFeedbackVisibility(true);
        m_pExViewer->setSceneGraph(sceneSep);
        m_pExViewer->viewAll();

        connect(UI->pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
        connect(UI->pushButtonLoadObject, SIGNAL(clicked()), this, SLOT(selectObject()));
        connect(UI->pushButtonSave, SIGNAL(clicked()), this, SLOT(saveObject()));
        connect(UI->pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
        connect(UI->pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
        connect(UI->pushButtonLoadRobot, SIGNAL(clicked()), this, SLOT(selectRobot()));
        connect(UI->comboBoxEEF, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));
        connect(UI->comboBoxGrasp, SIGNAL(activated(int)), this, SLOT(selectGrasp(int)));
        connect(UI->pushButtonAddGrasp, SIGNAL(clicked()), this, SLOT(addGrasp()));
        connect(UI->pushButtonRenameGrasp, SIGNAL(clicked()), this, SLOT(renameGrasp()));
        connect(UI->checkBoxTCP, SIGNAL(clicked()), this, SLOT(buildVisu()));

        connect(UI->horizontalSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectX()));
        connect(UI->horizontalSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectY()));
        connect(UI->horizontalSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectZ()));
        connect(UI->horizontalSliderRo, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectA()));
        connect(UI->horizontalSliderPi, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectB()));
        connect(UI->horizontalSliderYa, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectG()));
        connect(UI->checkBoxColModel, SIGNAL(clicked()), this, SLOT(buildVisu()));
        connect(UI->checkBoxGraspSet, SIGNAL(clicked()), this, SLOT(buildVisu()));


        // In case of embedded use of this program it should not be possible to load an object after the editor is started
        if (embeddedGraspEditor)
        {
            UI->pushButtonLoadObject->setVisible(false);
        }
    }

    QString GraspEditorWindow::formatString(const char* s, float f)
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


    void GraspEditorWindow::resetSceneryAll()
    {

    }


    void GraspEditorWindow::closeEvent(QCloseEvent* event)
    {
        quit();
        QMainWindow::closeEvent(event);
    }



    void GraspEditorWindow::buildVisu()
    {
        if (visualizationRobot)
        {
            visualizationRobot->highlight(false);
        }

        eefVisu->removeAllChildren();

        showCoordSystem();
        SceneObject::VisualizationType colModel = (UI->checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

        if (!UI->checkBoxTCP->isChecked())
        {
            if (robotEEF)
            {
                visualizationRobot = robotEEF->getVisualization<CoinVisualization>(colModel);
                SoNode* visualisationNode = visualizationRobot->getCoinVisualization();

                if (visualisationNode)
                {
                    eefVisu->addChild(visualisationNode);
                    //visualizationRobot->highlight(true);
                }
            }
        }
        else
        {
            if (robotEEF && robotEEF_EEF)
            {
                RobotNodePtr tcp = robotEEF_EEF->getTcp();

                if (tcp)
                {
                    SoSeparator* res = new SoSeparator;
                    eefVisu->addChild(res);
                    Eigen::Matrix4f tcpGP = tcp->getGlobalPose();
                    SoMatrixTransform* m = CoinVisualizationFactory::getMatrixTransformScaleMM2M(tcpGP);
                    res->addChild(m);
                    SoSeparator* co = CoinVisualizationFactory::CreateCoordSystemVisualization();
                    res->addChild(co);

                }
            }
        }

        objectSep->removeAllChildren();

        if (object)
        {
            SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(object, colModel);

            if (visualisationNode)
            {
                objectSep->addChild(visualisationNode);
            }
        }

        buildGraspSetVisu();
    }

    int GraspEditorWindow::main()
    {
        SoQt::show(this);
        SoQt::mainLoop();
        return 0;
    }


    void GraspEditorWindow::quit()
    {
        std::cout << "GraspEditorWindow: Closing" << std::endl;
        this->close();
        SoQt::exitMainLoop();
    }

    void GraspEditorWindow::selectRobot()
    {
        QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr("XML Files (*.xml)"));
        robotFile = std::string(fi.toAscii());
        loadRobot();
    }

    void GraspEditorWindow::selectObject(std::string file)
    {
        std::string s;

        // The object must be selected manually, cannot be done in the constructor
        if (embeddedGraspEditor)
        {
            s = file;
        }
        else
        {
            QString fi = QFileDialog::getOpenFileName(this, tr("Open ManipulationObject File"), QString(), tr("XML Files (*.xml)"));
            s = std::string(fi.toAscii());
        }

        if (s != "")
        {
            objectFile = s;
            loadObject();
        }
    }

    void GraspEditorWindow::saveObject()
    {
        if (!object)
        {
            return;
        }

        // No need to select a file where the object is saved, it is the same as the input file
        if (!embeddedGraspEditor)
        {
            QString fi = QFileDialog::getSaveFileName(this, tr("Save ManipulationObject"), QString(), tr("XML Files (*.xml)"));
            objectFile = std::string(fi.toAscii());
        }

        bool ok = false;

        try
        {
            ok = ObjectIO::saveManipulationObject(object, objectFile);
        }
        catch (VirtualRobotException& e)
        {
            cout << " ERROR while saving object" << endl;
            cout << e.what();
            return;
        }

        if (!ok)
        {
            cout << " ERROR while saving object" << endl;
            return;
        }
        else
        {
            if (embeddedGraspEditor)
            {
                cout << "Changes successful saved to " << objectFile << endl;
                QMessageBox msgBox;
                msgBox.setText(QString::fromStdString("Changes successful saved to " + objectFile));
                msgBox.setIcon(QMessageBox::Information);
                msgBox.setStandardButtons(QMessageBox::Ok);
                msgBox.setDefaultButton(QMessageBox::Ok);
                msgBox.exec();
            }
        }

    }
    void GraspEditorWindow::loadRobot()
    {
        robotSep->removeAllChildren();
        cout << "Loading Robot from " << robotFile << endl;

        try
        {
            robot = RobotIO::loadRobot(robotFile);
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

        robot->getEndEffectors(eefs);
        updateEEFBox();

        if (eefs.size() == 0)
        {
            selectEEF(-1);
        }
        else
        {
            selectEEF(0);
        }

        buildVisu();
        m_pExViewer->viewAll();
    }

    void GraspEditorWindow::selectEEF(int n)
    {
        currentEEF.reset();
        currentGraspSet.reset();
        currentGrasp.reset();

        eefVisu->removeAllChildren();
        robotEEF.reset();

        if (n < 0 || n >= (int)eefs.size() || !robot)
        {
            return;
        }

        currentEEF = eefs[n];

        currentEEF->print();

        robotEEF = currentEEF->createEefRobot(currentEEF->getName(), currentEEF->getName());
        //robotEEF->print();
        robotEEF_EEF = robotEEF->getEndEffector(currentEEF->getName());
        robotEEF_EEF->print();

        //bool colModel = UI.checkBoxColModel->isChecked();
        //eefVisu->addChild(CoinVisualizationFactory::getCoinVisualization(robotEEF,colModel));

        // select grasp set
        if (object)
        {
            currentGraspSet = object->getGraspSet(currentEEF);

            if (!currentGraspSet)
            {
                currentGraspSet.reset(new GraspSet(currentEEF->getName(), robot->getType(), currentEEF->getName()));
                currentGrasp.reset(new Grasp("Grasp 0", robot->getType(), currentEEF->getName(), Eigen::Matrix4f::Identity(), "GraspEditor"));
                currentGraspSet->addGrasp(currentGrasp);
                object->addGraspSet(currentGraspSet);
            }

            updateGraspBox();
            selectGrasp(0);
        }
        else
        {
            updateGraspBox();
        }

        buildVisu();

    }

    void GraspEditorWindow::selectGrasp(int n)
    {
        currentGrasp.reset();

        if (!currentGraspSet || n < 0 || n >= (int)currentGraspSet->getSize() || !robot)
        {
            return;
        }

        currentGrasp = currentGraspSet->getGrasp(n);

        if (currentGrasp && robotEEF && robotEEF_EEF && object)
        {
            Eigen::Matrix4f gp;
            gp = currentGrasp->getTransformation().inverse();
            gp = object->toGlobalCoordinateSystem(gp);
            std::string preshape = currentGrasp->getPreshapeName();

            if (!preshape.empty() && robotEEF_EEF->hasPreshape(preshape))
            {
                robotEEF_EEF->setPreshape(preshape);
            }

            setCurrentGrasp(gp);
        }

        buildVisu();
        m_pExViewer->scheduleRedraw();
    }

    void GraspEditorWindow::loadObject()
    {
        objectSep->removeAllChildren();
        cout << "Loading Object from " << objectFile << endl;

        try
        {
            object = ObjectIO::loadManipulationObject(objectFile);
        }
        catch (VirtualRobotException& e)
        {
            cout << " ERROR while creating object" << endl;
            cout << e.what();

            if (embeddedGraspEditor)
            {
                QMessageBox msgBox;
                msgBox.setText(QString::fromStdString(" ERROR while creating object."));
                msgBox.setInformativeText("Please select a valid manipulation file.");
                msgBox.setIcon(QMessageBox::Information);
                msgBox.setStandardButtons(QMessageBox::Ok);
                msgBox.setDefaultButton(QMessageBox::Ok);
                msgBox.exec();
            }

            return;
        }

        if (!object)
        {
            cout << " ERROR while creating object" << endl;
            return;
        }

        //object->print();

        selectEEF(0);

        buildVisu();
    }

    void GraspEditorWindow::updateEEFBox()
    {
        UI->comboBoxEEF->clear();

        for (size_t i = 0; i < eefs.size(); i++)
        {
            UI->comboBoxEEF->addItem(QString(eefs[i]->getName().c_str()));
        }
    }

    void GraspEditorWindow::updateGraspBox()
    {
        UI->comboBoxGrasp->clear();

        if (!currentGraspSet || currentGraspSet->getSize() == 0)
        {
            return;
        }

        for (unsigned int i = 0; i < currentGraspSet->getSize(); i++)
        {
            UI->comboBoxGrasp->addItem(QString(currentGraspSet->getGrasp(i)->getName().c_str()));
        }
    }

    void GraspEditorWindow::closeEEF()
    {
        if (robotEEF_EEF)
        {
            robotEEF_EEF->closeActors(object);
        }

        m_pExViewer->scheduleRedraw();
    }

    void GraspEditorWindow::openEEF()
    {
        if (robotEEF_EEF)
        {
            robotEEF_EEF->openActors();
        }

        m_pExViewer->scheduleRedraw();
    }

    void GraspEditorWindow::renameGrasp()
    {
        bool ok;
        QString text = QInputDialog::getText(this, tr("Rename Grasp"),
            tr("New name:"), QLineEdit::Normal,
            tr(currentGrasp->getName().c_str()), &ok);


        if (ok && !text.isEmpty())
        {
            std::string sText = text.toStdString();
            currentGrasp->setName(sText);

            updateGraspBox();
        }
    }

    void GraspEditorWindow::addGrasp()
    {
        if (!object || !robot || !currentGraspSet)
        {
            return;
        }

        std::stringstream ss;
        ss << "Grasp " << (currentGraspSet->getSize());
        std::string name = ss.str();
        Eigen::Matrix4f pose;

        if (currentGrasp)
        {
            pose = currentGrasp->getTransformation();
        }
        else
        {
            pose = Eigen::Matrix4f::Identity();
        }

        GraspPtr g(new Grasp(name, robot->getType(), currentEEF->getName(), pose, std::string("GraspEditor")));
        currentGraspSet->addGrasp(g);
        updateGraspBox();
        UI->comboBoxGrasp->setCurrentIndex(UI->comboBoxGrasp->count() - 1);
        selectGrasp(UI->comboBoxGrasp->count() - 1);
        buildVisu();
    }

    void GraspEditorWindow::updateEEF(float x[6])
    {
        if (robotEEF)
        {
            //cout << "getGlobalPose robot:" << endl << robotEEF->getGlobalPose() << endl;
            //cout << "getGlobalPose TCP:" << endl <<  robotEEF_EEF->getTcp()->getGlobalPose() << endl;
            Eigen::Matrix4f m;
            MathTools::posrpy2eigen4f(x, m);
            RobotNodePtr tcp = robotEEF_EEF->getTcp();
            m = tcp->getGlobalPose() * m;
            //cout << "pose:" << endl << m << endl;
            setCurrentGrasp(m);
        }

        m_pExViewer->scheduleRedraw();
    }

    void GraspEditorWindow::sliderReleased_ObjectX()
    {
        UI->horizontalSliderX->setValue(0);
    }

    void GraspEditorWindow::sliderReleased_ObjectY()
    {
        UI->horizontalSliderY->setValue(0);
    }

    void GraspEditorWindow::sliderReleased_ObjectZ()
    {
        UI->horizontalSliderZ->setValue(0);
    }

    void GraspEditorWindow::sliderReleased_ObjectA()
    {
        UI->horizontalSliderRo->setValue(0);
    }

    void GraspEditorWindow::sliderReleased_ObjectB()
    {
        UI->horizontalSliderPi->setValue(0);
    }

    void GraspEditorWindow::sliderReleased_ObjectG()
    {
        UI->horizontalSliderYa->setValue(0);
    }


    void GraspEditorWindow::setCurrentGrasp(Eigen::Matrix4f& p)
    {
        if (robotEEF && robotEEF_EEF && currentGrasp && object)
        {
            RobotNodePtr tcp = robotEEF_EEF->getTcp();
            robotEEF->setGlobalPoseForRobotNode(tcp, p);
            Eigen::Matrix4f objP = object->getGlobalPose();
            Eigen::Matrix4f pLocal = tcp->toLocalCoordinateSystem(objP);
            currentGrasp->setTransformation(pLocal);
        }

        m_pExViewer->scheduleRedraw();
    }

    void GraspEditorWindow::showCoordSystem()
    {
        if (robotEEF && robotEEF_EEF)
        {
            RobotNodePtr tcp = robotEEF_EEF->getTcp();

            if (!tcp)
            {
                return;
            }

            tcp->showCoordinateSystem(UI->checkBoxTCP->isChecked());
        }

        if (object)
        {
            object->showCoordinateSystem(UI->checkBoxTCP->isChecked());
        }
    }


    void GraspEditorWindow::buildGraspSetVisu()
    {
        graspSetVisu->removeAllChildren();

        if (UI->checkBoxGraspSet->isChecked() && robotEEF && robotEEF_EEF && currentGraspSet && object)
        {
            GraspSetPtr gs = currentGraspSet->clone();
            gs->removeGrasp(currentGrasp);
            SoSeparator* visu = CoinVisualizationFactory::CreateGraspSetVisualization(gs, robotEEF_EEF, object->getGlobalPose());

            if (visu)
            {
                graspSetVisu->addChild(visu);
            }
        }
    }

}