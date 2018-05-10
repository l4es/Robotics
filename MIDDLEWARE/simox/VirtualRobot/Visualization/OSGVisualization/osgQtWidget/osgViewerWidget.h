/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_OSGQtWidget_h_
#define _VirtualRobot_OSGQtWidget_h_


#include "../../../VirtualRobotImportExport.h"

#include <QtCore/QTimer>
#include <QtGui/QApplication>
#include <QtGui/QGridLayout>
#include <QtGui/QKeyEvent>

#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>

#include <osgDB/ReadFile>

#include <osgQt/GraphicsWindowQt>

#include <iostream>

#include <QGLWidget>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>

namespace VirtualRobot
{
    /*!
        A Qt widget showing an OpenSceneGraph scene. Have a look on the source code of RobtoViewerOSG for usage.
        To use this widget follow these instructions:
        * compile Simox with OSG support (and disable Coin support)
        * Create a Qt-4 Window with QtDesigner where an empty QFrame is provided for embedding this widget
        * In your C++ file,
        ** Load or create a osg scene:
            <code>
            robot = RobotIO::loadRobot(robotFilename,RobotIO::eFull);
            visualization = robot->getVisualization<OSGVisualization>(false);
            visualisationNode = visualization->getOSGVisualization();
            osgRobot->addChild(visualisationNode);
            osgRoot->addChild(osgRobot);
            </code>
        ** Embed an instance of this widget with:
            <code>
            osgWidget = new osgViewerWidget(osgRoot,UI.frameViewer);
            </code>
        ** Now the scene will display your robot, and you can add or remove osg::Nodes to/from osgRoot
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT osgViewerWidget: public QGLWidget
    {
        Q_OBJECT

    public:
        explicit osgViewerWidget(osg::Node* scene, QWidget* parent = 0);

        void viewAll();
    public slots:

        void timerCB();

    protected:
        virtual void initializeGL();
        virtual void resizeGL(int width, int height);

        virtual void paintGL();

        void resizeEvent(QResizeEvent* event);

        virtual void keyPressEvent(QKeyEvent* event);
        virtual void keyReleaseEvent(QKeyEvent* event);
        virtual void mousePressEvent(QMouseEvent* event);
        virtual void mouseReleaseEvent(QMouseEvent* event);
        virtual void mouseMoveEvent(QMouseEvent* event);
        virtual void mouseDoubleClickEvent(QMouseEvent* event);
        virtual void wheelEvent(QWheelEvent* event);

        void paintOSG();
        void resizeOSG(int width, int height);

    protected:

        osg::ref_ptr<osgViewer::Viewer> viewer;
        osg::observer_ptr<osgViewer::GraphicsWindowEmbedded> window;
        osg::ref_ptr<osg::Node> loadedModel;
        osg::ref_ptr<osg::MatrixTransform> transformation;
        osgGA::TrackballManipulator* sceneManipulator;

        //osgQt::GraphicsWindowQt* m_qt_win;

        QTimer paintTimer;
        QSize canvasSize;
    };

} // namespace VirtualRobot
/*
class osgViewerWidget : public QWidget, public osgViewer::CompositeViewer
{
public:
    osgViewerWidget(osg::Node* scene, QWidget* parent = NULL, osgViewer::ViewerBase::ThreadingModel threadingModel=osgViewer::CompositeViewer::SingleThreaded);



    osg::Camera* createCamera( int x, int y, int w, int h, const std::string& name="", bool windowDecoration=false );


    virtual void paintEvent( QPaintEvent* event );
    QWidget* getQWidget();

protected:
    osgViewer::View* view;
    osg::Camera* camera;
    QTimer _timer;
};*/


#endif // _VirtualRobot_OSGQtWidget_h_
