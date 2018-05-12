/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi (daniele.calisi@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#define MODULE_NAME "GuiManagerModule"

#include "guimanagermodule.h"
#include "../../../ragent2/main/ragent.h"
#include "../../rqcommon/rqcommon.h"
#include "../../rqcommon/rguiobjects.h"
#include <rdkcore/config.h>
#include <rdkcore/filesystem/filesystem.h>

#include <rdkcore/logging/logging.h>
#include <rdkcore/robot/robotmodule.h>
#define LOGGING_MODULE MODULE_NAME

#include <qlabel.h>
#include <qlayout.h>
#include <qdragobject.h>

//#include <X11/X.h>
//#include <X11/Xlib.h>

extern RDK2::RAgent::RAgent ragent;

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RAgent;

bool GuiManagerModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		Common::createDefaultProperties(session, true, false);
		session->createStorage("RGuiState", "guiState", "RConsole GUI state");
		session->setPersistent("guiState");
		session->setObject("guiState", new RGuiState());
	SESSION_END_CATCH_TERMINATE(session)

	return true;
}

bool GuiManagerModule::init()
{
//	if (XInitThreads()) { RDK_INFO_PRINTF("X multi-threading is ON"); }
//	else { RDK_ERROR_PRINTF("Cannot set multi-threading for X"); }

	RqCommon::registerViewer("RMotionParams", "rdkrqm_genericstringviewermodule", "Show with default string viewer", true);
	RqCommon::registerViewer("RBool", "rdkrqm_boolviewermodule", "Show with default bool viewer", true);
	RqCommon::registerViewer("RInt", "rdkrqm_intviewermodule", "Show with default int viewer", true);
	RqCommon::registerViewer("RDouble", "rdkrqm_doubleviewermodule", "Show with default double viewer", true);
	RqCommon::registerViewer("RString", "rdkrqm_stringviewermodule", "Show with default string viewer (default)", true);
	RqCommon::registerViewer("RString", "rdkrqm_textviewermodule", "Show with text viewer");
	RqCommon::registerViewer("RString", "rdkrqm_filenamechoosermodule", "Show with filename chooser");
	RqCommon::registerViewer("RPoint2od", "rdkrqm_poseviewermodule", "Show with default pose (point2od) viewer", true);
	RqCommon::registerViewer("RPoint2odCov", "rdkrqm_point2odcovviewermodule", "Show with default point2od (with covariance) viewer", true);
	RqCommon::registerViewer("RPolarPoint", "rdkrqm_polarpointviewermodule", "Show with default polarpoint viewer", true);
	RqCommon::registerViewer("RPolarPointCov", "rdkrqm_polarpointcovviewermodule", "Show with default polarpoint (with covariance) viewer", true);
	RqCommon::registerViewer("RPoint2d", "rdkrqm_point2dviewermodule", "Show with default point2d viewer", true);
	RqCommon::registerViewer("RPoint2dCov", "rdkrqm_point2dcovviewermodule", "Show with default point2d (with covariance) viewer", true);
	RqCommon::registerViewer("RC8Set", "rdkrqm_c8setviewermodule", "Show with default c8 set viewer", true);
	RqCommon::registerViewer("RMapImage", "rdkrqm_mapviewerglmodule", "Show with OpenGL viewer (default)", true);
	RqCommon::registerViewer("RMapImage", "rdkrqm_mapviewerqtmodule", "Show with QT viewer");
	RqCommon::registerViewer("RImage", "rdkrqm_imageviewerglmodule", "Show with OpenGL viewer (default)", true);
// 	RqCommon::registerViewer("RImage", "rdkrqm_imageviewerqtmodule", "Show with QT viewer");
	RqCommon::registerViewer("RYellowPages", "rdkrqm_yellowpagesmodule", "Show with default yellow pages viewer", true);
	RqCommon::registerViewer("RObjectPipeQueue", "rdkrqm_queueviewermodule", "Show with default Queue stats viewer", true);

	RqCommon::registerTool("rdkrqm_cmdsendermodule", "Command sender");
	RqCommon::registerTool("-", "-");
	RqCommon::registerTool("rdkrqm_fourarrowsmodule", "4-button controller");
	RqCommon::registerTool("rdkrqm_joystickmodule", "Joystick controller");
	RqCommon::registerTool("rdkrqm_eightarrowsmodule", "8-button controller");
	RqCommon::registerTool("rdkrqm_pantiltarrowsmodule", "Pan-tilt controller");

	//vector<string> dirs;
	//string scmd = string() + OpenRDK_OUTPUT_TREE;
	//if (RDK2::Filesystem::fsDirContent(scmd,dirs))
	//{
	//  for (vector<string>::const_iterator dit=dirs.begin();
	//       dit!=dirs.end();
	//       ++dit)
	//  {
	//    string libdir = scmd + "/" + *dit;
	//    if (dit->compare(0,3,"lib") == 0)
	//    {
	//      vector<string> libs;
	//      if (RDK2::Filesystem::fsDirContent(libdir,libs))
	//      {
	//        for (vector<string>::const_iterator lit=libs.begin();
	//             lit!=libs.end();
	//             ++lit)
	//        {
	//          string lib = libdir + "/" + *lit;
	//          if (lit->find("rdkrqm") != string::npos && lit->find("guimanager") == string::npos)
	//          {
	//            RDK_DEBUG_STREAM("Found rq plugin: " << lib);
	//            //Module* plugin = Module::createFromLibrary(lib);
	//            //RDK_WARNING_STREAM("Desc:"     << plugin->getDescription());
	//            //RDK_WARNING_STREAM("Libname: " << plugin->getModuleName());
	//            //if (plugin->isHandler())
	//            // RDK_WARNING_STREAM("Handle: " << plugin->getHandleName());
	//          }
	//        }
	//      }
	//      else
	//      {
	//        RDK_WARNING_STREAM("Error [" << libdir << "] is not a valid dir");
	//      }
	//    }
	//  }
	//}
	//else
	//{
	//  RDK_WARNING_STREAM("Error scanning [" << scmd << "] while looking for rconsoleqt plugins");
	//}

	bool guiEnabled = true;
	int argc = 0; char **argv = {0};
	app = new QApplication(argc, argv, guiEnabled);

	mainWindow = new RqMainWindow();
	app->setMainWidget(mainWindow);
	RqCommon::setMainWindow(mainWindow);
	RqCommon::setModuleManager(getModuleManager());
	RqCommon::setGuiManagerName(getModuleName());

	mainWindow->setCaption((string() + "RConsole-QT (Agent name '" + session->getRepositoryName() + "')").c_str());

	return true;
}

void GuiManagerModule::exec()
{
	RDK_DEBUG_PRINTF("QT thread ID: %p", pthread_self());
	RqCommon::setQtThread();
	RqCommon::createWindowEventFilter();
	RqCommon::registerGuiSession(session);

	mainWindow->show();

	// first refresh: rconsole qt modules that was in the configuration file will have their window/widgets showed
	RqCommon::refreshGui();
	RDK_DEBUG_PRINTF("Starting QT application");
	int result = app->exec();
	RDK_INFO_STREAM("Exited QT event loop (result = " << result << ")");
	RqCommon::getGuiSession(getModuleName());
	ragent.quitSemaphore.signal();
}

void GuiManagerModule::exitRequested()
{
	app->exit();
}

MODULE_FACTORY(GuiManagerModule);

}} // namespace
