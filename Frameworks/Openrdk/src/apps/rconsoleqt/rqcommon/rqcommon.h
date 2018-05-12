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

#ifndef RDK2_RCONSOLE_QT_RQCOMMON
#define RDK2_RCONSOLE_QT_RQCOMMON

#include <set>
#include <map>
#include <pthread.h>
#include <qevent.h>
#include <rdkcore/posixconstructs/posixmutex.h>
#include <rdkcore/repository/session.h>
#include <rdkcore/modules/modulemanager.h>

#include "rqmainwindow.h"

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RepositoryNS;
using namespace PosixConstructs;
using namespace RDK2::RAgent;
using namespace std;

class RcWindowEventFilter : public QObject {
public:
	bool eventFilter(QObject* obj, QEvent* evt);
};

class RqCommon {
public:
	static const int EVENT_CREATE_WIDGET = QEvent::User + 1;
	static const int EVENT_DESTROY_WIDGET = QEvent::User + 2;
	static const int EVENT_LOCAL = QEvent::User + 3;
	
	struct RegisteredTool {
		string menuCaption, libraryName;
		inline bool operator<(const RegisteredTool& r) const
		{
			return menuCaption < r.menuCaption || (menuCaption == r.menuCaption && libraryName < r.libraryName);
		}
	};
	
	struct RegisteredViewer {
		string targetClass, menuCaption, libraryName;
		bool defaultViewer;
		inline bool operator<(const RegisteredViewer& r) const
		{
			return targetClass < r.targetClass
				|| (targetClass == r.targetClass &&
					(menuCaption < r.menuCaption
					|| (menuCaption == r.menuCaption 
					&& libraryName < r.libraryName)));
		}
	};
	
	static void registerTool(const string& libraryName, const string& menuCaption);
	static void registerViewer(const string& targetClass, const string& libraryName, const string& menuCaption, bool defaultViewer = false);
	static set<RegisteredViewer> getAllViewers();
	static set<RegisteredViewer> getViewersForClass(const string& className);
	static vector<RegisteredTool> getAllTools();
	static bool registerGuiSession(Session* session);
	inline static Session* getGuiSession(const string& moduleName) { guiSession->setUrlContext("/" + moduleName); return guiSession; }
	
	static void registerModuleWidget(const string& moduleName, QWidget* widget);
	static QWidget* getModuleWidget(const string& moduleName);
	static void unregisterModuleWidget(const string& moduleName);
	
	static void createWidget(const string& namePrefix, const string& libraryName, const string& textConfig,
		const QPoint& p, const QSize& s, int parentId = -1);
	
	//static void createTool(const string& libraryName, const QPoint& p, const QSize& s, int parentId = -1);
	static void createTreeWidget(const string& agentName);
	static void createDefaultViewerFor(const string& propertyUrl, const QPoint& p, int parentId = -1);
	static void createViewerFor(const string& propertyUrl, const string& libraryName, const QPoint& p, int parentId = -1);
	
	inline static void setModuleManager(ModuleManager* mm) { moduleManager = mm; }
	inline static ModuleManager* getModuleManager() { return moduleManager; }
	
	inline static void setMainWindow(RqMainWindow* mw) { mainWindow = mw; }
	inline static RqMainWindow* getMainWindow() { return mainWindow; }
	
	inline static void setQtThread() { qtThreadId = pthread_self(); }
	inline static bool checkQtThread() { return pthread_equal(qtThreadId, pthread_self()); }
	
	inline static void setGuiManagerName(const string& name) { guiManagerName = name; }
	inline static string getGuiManagerName() { return guiManagerName; }
	inline static string getGuiStateUrl() { return "/" + guiManagerName + "/" + "guiState"; }
	
	inline static void createWindowEventFilter() { windowEventFilter = new RcWindowEventFilter(); }
	
	static void refreshGui();
	static void updateGuiState();
	
protected:
	static Session* guiSession;
	static PosixMutex mutex;
	static vector<RegisteredTool> registeredTools;
	static set<RegisteredViewer> registeredViewers;
	static map<string, QWidget*> moduleWidgetMap;
	static ModuleManager* moduleManager;
	static RqMainWindow* mainWindow;
	static pthread_t qtThreadId;
	static string guiManagerName;
	static RcWindowEventFilter* windowEventFilter;
};

// use this macro in functions that have to be called by QT thread
#define QT_THREAD_GUARD(F) if (exiting) return F; if (!RqCommon::checkQtThread()) { RDK_ERROR_PRINTF("Not in QT thread (%p)!!", pthread_self()); return F; }

// put this macro before the while loop in the exec() of your rconsole module
#define WIDGET_GUARD while (!widgetReady) { usleep(20000); }

}}

#endif
