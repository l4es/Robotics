#include "osgViewerWidget.h"

#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

namespace VirtualRobot
{

    osgViewerWidget::osgViewerWidget(osg::Node* scene, QWidget* parent) : QGLWidget(parent)
    {
        viewer = new osgViewer::Viewer;
        transformation = new osg::MatrixTransform;

        if (scene)
        {
            transformation->addChild(scene);
        }

        paintTimer.setInterval(20);
        canvasSize.setWidth(200);
        canvasSize.setHeight(200);

        if (parent)
        {
            canvasSize = parent->size();
        }
    }

    void osgViewerWidget::initializeGL()
    {
        window = viewer->setUpViewerAsEmbeddedInWindow(0, 0, canvasSize.width(), canvasSize.height());
        viewer->addEventHandler(new osgGA::StateSetManipulator(transformation->getOrCreateStateSet()));
        viewer->addEventHandler(new osgViewer::StatsHandler);
        viewer->addEventHandler(new osgViewer::WindowSizeHandler);
        sceneManipulator = new osgGA::TrackballManipulator;
        sceneManipulator->setAutoComputeHomePosition(true);
        viewer->setCameraManipulator(sceneManipulator);
        viewer->setSceneData(transformation.get());

        connect(&paintTimer, SIGNAL(timeout()), this, SLOT(timerCB()));
        paintTimer.start();
    }

    void osgViewerWidget::resizeGL(int width, int height)
    {
        // this method is not invoked on when resizing the window?!
        std::cout << "resizeGL " << width << "," << height << std::endl;
        resizeOSG(width, height);
    }

    void osgViewerWidget::resizeOSG(int width, int height)
    {
        if (window.valid() && parentWidget())
        {
            //std::cout << "Resizing " << width << "," << height << std::endl;
            window->resized(window->getTraits()->x, window->getTraits()->y, width, height);
            window->getEventQueue()->windowResize(window->getTraits()->x, window->getTraits()->y, width, height);
            // the qt GLwidget must also be resized
            resize(width, height);
            //window = viewer->setUpViewerAsEmbeddedInWindow(0, 0, width, height);
        }
    }

    void osgViewerWidget::timerCB()
    {
        if (parentWidget())
        {
            // since resizeGL is not called we need this little hack to ensure correct resizing of our widget
            if (canvasSize != parentWidget()->size() &&
                QApplication::mouseButtons() == Qt::NoButton)
            {
                //std::cout << "Resizing..." << std::endl;
                resizeOSG(parentWidget()->size().width(), parentWidget()->size().height());
                canvasSize = parentWidget()->size();
            }
        }

        paintOSG();

    }
    void osgViewerWidget::paintOSG()
    {
        if (viewer.valid())
        {
            makeCurrent();
            viewer->frame();
            swapBuffers();
        }
    }

    void osgViewerWidget::paintGL()
    {
        paintOSG();

    }

    //////////////////////////////////////////////////////////////////////////////////
    void osgViewerWidget::keyPressEvent(QKeyEvent* event)
    {
        if (window.valid())
        {
            //int value = STATIC_KEY_MAP.remapKey(event);
            //window->getEventQueue()->keyPress( value );
        }
    }

    //////////////////////////////////////////////////////////////////////////////////
    void osgViewerWidget::keyReleaseEvent(QKeyEvent* event)
    {
        if (window.valid())
        {
            //int value = STATIC_KEY_MAP.remapKey(event);
            //window->getEventQueue()->keyRelease( value );
        }
    }

    //////////////////////////////////////////////////////////////////////////////////
    void osgViewerWidget::mousePressEvent(QMouseEvent* event)
    {
        int button = 0;

        switch (event->button())
        {
            case (Qt::LeftButton):
                button = 1;
                break;

            case (Qt::MidButton):
                button = 2;
                break;

            case (Qt::RightButton):
                button = 3;
                break;

            case (Qt::NoButton):
                button = 0;
                break;

            default:
                button = 0;
                break;
        }

        if (window.valid())
        {
            window->getEventQueue()->mouseButtonPress(event->x(), event->y(), button);
        }

    }

    //////////////////////////////////////////////////////////////////////////////////
    void osgViewerWidget::mouseReleaseEvent(QMouseEvent* event)
    {
        int button = 0;

        switch (event->button())
        {
            case (Qt::LeftButton):
                button = 1;
                break;

            case (Qt::MidButton):
                button = 2;
                break;

            case (Qt::RightButton):
                button = 3;
                break;

            case (Qt::NoButton):
                button = 0;
                break;

            default:
                button = 0;
                break;
        }

        if (window.valid())
        {
            window->getEventQueue()->mouseButtonRelease(event->x(), event->y(), button);
        }
    }

    //////////////////////////////////////////////////////////////////////////////////
    void osgViewerWidget::mouseMoveEvent(QMouseEvent* event)
    {
        if (window.valid())
        {
            window->getEventQueue()->mouseMotion(event->x(), event->y());
        }
    }

    //////////////////////////////////////////////////////////////////////////
    void osgViewerWidget::mouseDoubleClickEvent(QMouseEvent* event)
    {
        int button = 0;

        switch (event->button())
        {
            case (Qt::LeftButton):
                button = 1;
                break;

            case (Qt::MidButton):
                button = 2;
                break;

            case (Qt::RightButton):
                button = 3;
                break;

            case (Qt::NoButton):
                button = 0;
                break;

            default:
                button = 0;
                break;
        }

        if (window.valid())
        {
            window->getEventQueue()->mouseDoubleButtonPress(event->x(), event->y(), button);
        }
    }

    //////////////////////////////////////////////////////////////////////////////////
    void osgViewerWidget::wheelEvent(QWheelEvent* event)
    {
        if (window.valid())
        {
            if (event->orientation() == Qt::Horizontal)
            {
                if (event->delta() > 0)
                {
                    window->getEventQueue()->mouseScroll(osgGA::GUIEventAdapter::SCROLL_LEFT);
                }
                else
                {
                    window->getEventQueue()->mouseScroll(osgGA::GUIEventAdapter::SCROLL_RIGHT);
                }
            }
            else
            {
                if (event->delta() > 0)
                {
                    window->getEventQueue()->mouseScroll(osgGA::GUIEventAdapter::SCROLL_UP);
                }
                else
                {
                    window->getEventQueue()->mouseScroll(osgGA::GUIEventAdapter::SCROLL_DOWN);
                }
            }
        }
    }

    void osgViewerWidget::resizeEvent(QResizeEvent* event)
    {
        QSize widgetSize = event->size();
        std::cout << "w:" << widgetSize.width() << ", h:" << widgetSize.height() << std::endl;

        QGLWidget::resizeEvent(event);
    }

    void osgViewerWidget::viewAll()
    {
        if (!sceneManipulator)
        {
            return;
        }

        sceneManipulator->computeHomePosition();
        sceneManipulator->home(0);
    }


    /*

    osgViewerWidget::osgViewerWidget(osg::Node* scene, QWidget* parent, osgViewer::ViewerBase::ThreadingModel threadingModel) : QWidget(parent)
    {
        setThreadingModel(threadingModel);

        //camera = createCamera(0,0,200,200,"OSGViewer");
        view = new osgViewer::View;
        //view->setCamera( camera );
        camera = view->getCamera();

        addView( view );
        view->setSceneData( scene );
        view->addEventHandler( new osgViewer::StatsHandler );
        view->setCameraManipulator( new osgGA::TrackballManipulator );
        view->setUpViewerAsEmbeddedInWindow(50, 50, 1024, 768);
        view->getEventQueue()->windowResize(0, 0, 1024, 768);

        QVBoxLayout *vbox = new QVBoxLayout(this);
        vbox->addWidget(getQWidget());
        setLayout( vbox );

        this->show();

        connect( &_timer, SIGNAL(timeout()), this, SLOT(update()) );
        _timer.start( 10 );
    }

    QWidget* osgViewerWidget::getQWidget()
    {
        if (!camera)
            return NULL;
        osgQt::GraphicsWindowQt* gw = dynamic_cast<osgQt::GraphicsWindowQt*>( camera->getGraphicsContext() );
        return gw ? gw->getGLWidget() : NULL;
    }

    osg::Camera* osgViewerWidget::createCamera( int x, int y, int w, int h, const std::string& name, bool windowDecoration )
    {
        osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->windowName = name;
        traits->windowDecoration = windowDecoration;
        traits->x = x;
        traits->y = y;
        traits->width = w;
        traits->height = h;
        traits->doubleBuffer = true;
        traits->alpha = ds->getMinimumNumAlphaBits();
        traits->stencil = ds->getMinimumNumStencilBits();
        traits->sampleBuffers = ds->getMultiSamples();
        traits->samples = ds->getNumMultiSamples();

        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );

        camera->setClearColor( osg::Vec4(0.2, 0.2, 0.6, 1.0) );
        camera->setViewport( new osg::Viewport(0, 0, traits->width, traits->height) );
        camera->setProjectionMatrixAsPerspective(
            30.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), 1.0f, 10000.0f );
        return camera.release();
    }

    void osgViewerWidget::paintEvent( QPaintEvent* event )
    {
        frame();
    }
    */
} // namespace
