#include "BulletOpenGLViewer.h"

namespace SimDynamics
{


    BulletOpenGLViewer::BulletOpenGLViewer(DynamicsWorldPtr world)
    {
        SIMDYNAMICS_ASSERT(world);
        m_sundirection = btVector3(1, 1, -2) * 1000;

        bulletEngine = boost::dynamic_pointer_cast<BulletEngine>(world->getEngine());

        SIMDYNAMICS_ASSERT(bulletEngine);

        setTexturing(true);
        setShadows(true);

        // set Bullet world (defined in bullet's OpenGL helpers)
        m_dynamicsWorld = bulletEngine->getBulletWorld();
        m_dynamicsWorld->setDebugDrawer(&debugDrawer);

        // set up vector for camera
        setCameraDistance(btScalar(3.));
        setCameraForwardAxis(1);
        btVector3 up(0, 0, btScalar(1.));
        setCameraUp(up);

        clientResetScene();
    }

    void BulletOpenGLViewer::clientMoveAndDisplay()
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //simple dynamics world doesn't handle fixed-time-stepping
        double ms = getDeltaTimeMicroseconds();

        double minFPS = 1000000.f / 60.f;

        if (ms > minFPS)
        {
            ms = minFPS;
        }

        if (m_dynamicsWorld)
        {
            btScalar dt1 = btScalar(ms / 1000000.0f);

            m_dynamicsWorld->stepSimulation(dt1, 4);

            //optional but useful: debug drawing
            m_dynamicsWorld->debugDrawWorld();
        }

        renderme();

        glFlush();

        swapBuffers();
    }

    void BulletOpenGLViewer::displayCallback()
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (m_dynamicsWorld)
        {
            m_dynamicsWorld->debugDrawWorld();
        }

        renderme();

        glFlush();
        swapBuffers();
    }

    void BulletOpenGLViewer::keyboardCallback(unsigned char key, int x, int y)
    {
        DemoApplication::keyboardCallback(key, x, y);
        /*switch (key)
        {
        case 'e':
            spawnRagdoll(true);
            break;
        default:
            DemoApplication::keyboardCallback(key, x, y);
        }*/
    }

    BulletOpenGLViewer::~BulletOpenGLViewer()
    {

    }

    void BulletOpenGLViewer::initPhysics()
    {
        // nothing to do, this has already be done in DynamicsWorld

    }

    void BulletOpenGLViewer::myinit(void)
    {

        GLfloat light_ambient[] = { btScalar(0.2), btScalar(0.2), btScalar(0.2), btScalar(1.0) };
        GLfloat light_diffuse[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0) };
        GLfloat light_specular[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0)};
        /*  light_position is NOT default value */
        GLfloat light_position0[] = { btScalar(1.0), btScalar(1.0), btScalar(10.0), btScalar(0.0)};
        GLfloat light_position1[] = { btScalar(-1.0), btScalar(-1.0), btScalar(-10.0), btScalar(0.0) };

        glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
        glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
        glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

        glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
        glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
        glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
        glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHT1);


        glShadeModel(GL_SMOOTH);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        glClearColor(btScalar(0.7), btScalar(0.7), btScalar(0.7), btScalar(0));

        //  glEnable(GL_CULL_FACE);
        //  glCullFace(GL_BACK);
    }

    void BulletOpenGLViewer::enableContraintsDebugDrawing()
    {
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits | btIDebugDraw::DBG_DrawContactPoints);

    }


}
