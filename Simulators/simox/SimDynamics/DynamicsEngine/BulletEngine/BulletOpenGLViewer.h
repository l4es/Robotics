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
* @package    SimDynamics
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#ifndef _SimDynamics_BulletOpenGLViewer_h_
#define _SimDynamics_BulletOpenGLViewer_h_

#include "../../SimDynamics.h"
#include "../../DynamicsWorld.h"
#include "BulletEngine.h"

#ifdef _WINDOWS
#include "BulletOpenGL/Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "BulletOpenGL/GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

#include <btBulletDynamicsCommon.h>
#include "BulletOpenGL/GlutStuff.h"
#include "BulletOpenGL/GL_ShapeDrawer.h"
#include <LinearMath/btIDebugDraw.h>
#include "BulletOpenGL/GLDebugDrawer.h"

#ifdef _WIN32
#pragma warning(disable: 4275)
#endif

namespace SimDynamics
{

    class SIMDYNAMICS_IMPORT_EXPORT BulletOpenGLViewer : public PlatformDemoApplication
    {
    public:
        BulletOpenGLViewer(DynamicsWorldPtr world);
        virtual ~BulletOpenGLViewer();

        virtual void clientMoveAndDisplay();
        virtual void displayCallback();
        virtual void keyboardCallback(unsigned char key, int x, int y);
        virtual void initPhysics();
        virtual void myinit();

        virtual void enableContraintsDebugDrawing();
    protected:


        /*static DemoApplication* Create()
        {
            BulletOpenGLViewer* demo = new BulletOpenGLViewer;
            demo->myinit();
            demo->initPhysics();
            return demo;
        }*/

        void updateRobotConstraints();
        GLDebugDrawer debugDrawer;

        BulletEnginePtr bulletEngine;
    };


    typedef boost::shared_ptr<BulletOpenGLViewer> BulletOpenGLViewerPtr;

} // namespace

#endif // _SimDynamics_BulletOpenGLViewer_h_
