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

#include <pthread.h>
#include <qtabwidget.h>
#include <qlayout.h>
#include <qapplication.h>

#include <rdkcore/modules_config/moduleconfig.h>
#include <rdkcore/modules/module.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RqCommon"

#include "rqcommon.h"
#include "rguiobjects.h"
#include "rqtabwidget.h"
#include "rqmainwindow.h"

//#define DEBUG_THIS
#ifdef DEBUG_THIS
#define DEBUG_PRINTF(s, a...) RDK_DEBUG_PRINTF(s, ##a)
#else
#define DEBUG_PRINTF(s, a...)
#endif

/*

normal sequence:
1) some event make calling RqCommon::createWidget() [RqCommon, QT thread]
2) RqCommon::createWidget() calls instantiateModule() [RqCommon, QT thread]
3) the new module is instantiated, its init() called [Module, QT thread]
4) the module's init() post an event (in this case is unuseful, because we are already in QT thread) [Module, QT thread]
5) the module's name is added to the guiState property [Module, QT thread]
6) RqCommon::refreshGui is called [Module, QT thread]
7) refreshGui calls QApplication::processEvents, in order for the widget creation to be executed [RqCommon, QT thread]
8) module's createWidget is called and registered [Module, QT thread]
9) the widget is displayed

initialization sequence (when rconsole starts):
1) module is initialized [Module, some thread]
2) module's init() post an event [Module, some thread]
3) every modules are started, then guimanager do exec(), calling RqCommon::refreshGui [QT thread (guimanager thread)]
4) refreshGui calls QApplication::processEvents, in order for the widget creation to be executed [RqCommon, QT thread]
5) module's createWidget is called and registered [Module, QT thread]
6) the widget is displayed

other notes:
- please put the QT_THREAD_GUARD macro in the beginning of all functions that should be called in the QT thread
- please ALWAYS build a widget in the QT thread, NEVER in other threads; the same is for everything that has to do with QT objects
- please put the WIDGET_GUARD macro BEFORE the while loop in the exec() of your module: otherwise the presence of the widget
  is not ensured
- please always initialize the bool widgetReady to false in the module constructor (otherwise the above will not work)
  and remember to set it true when the widget has been completely built (after the createWidget function call, usually)

for these reasons, each rqconsole module have to:
- have a bool widgetReady; member set to false in the module constructor
- put QApplication::postEvent(this, new QCustomEvent(RqCommon::EVENT_CREATE_WIDGET)); in its init() function
- put QApplication::postEvent(this, new QCustomEvent(RqCommon::EVENT_DESTROY_WIDGET)); in its cleanup() function
- implement void customEvent(QCustomEvent* e) (better if private or protected), that should be like this:
	if (e->type() == RqCommon::EVENT_CREATE_WIDGET) {
		RqCommon::registerModuleWidget(getModuleName(), createWidget());
		widgetReady = true;
	}
	else if (e->type() == RqCommon::EVENT_DESTROY_WIDGET) {
		RqCommon::unregisterModuleWidget(getModuleName());
		moduleWidget->close(true);
	}
- implement QWidget* createWidget() that creates the module widget and returns it
- put WIDGET_GUARD before the while loop in the exec() function of the module, or anyway BEFORE any reference to the widget
- put QT_THREAD_GUARD as the first command in almost any function that you suppose need to be called from the QT thread (i.e.,
  all those that reference a QT object, or create it, or manipulate, or call QT functions, etc.)
- never call QT functions from the exec() function, because they have to be called from the QT thread; a common workaround
  is the use of QApplication::postEvent function

*/

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RAgent;

Session* RqCommon::guiSession = 0;
PosixMutex RqCommon::mutex;
vector<RqCommon::RegisteredTool> RqCommon::registeredTools;
set<RqCommon::RegisteredViewer> RqCommon::registeredViewers;
map<string, QWidget*> RqCommon::moduleWidgetMap;
ModuleManager* RqCommon::moduleManager = 0;
RqMainWindow* RqCommon::mainWindow = 0;
pthread_t RqCommon::qtThreadId = 0;
string RqCommon::guiManagerName;
RcWindowEventFilter* RqCommon::windowEventFilter = 0;

void RqCommon::registerTool(cstr libraryName, cstr menuCaption)
{
	RegisteredTool rt;
	rt.menuCaption = menuCaption;
	rt.libraryName = libraryName;
	mutex.lock(HERE);
	registeredTools.push_back(rt);
	mutex.unlock();
}

void RqCommon::registerViewer(cstr targetClass, cstr libraryName, cstr menuCaption, bool defaultViewer)
{
	RegisteredViewer rv;
	rv.targetClass = targetClass;
	rv.menuCaption = menuCaption;
	rv.libraryName = libraryName;
	rv.defaultViewer = defaultViewer;
	mutex.lock(HERE);
	registeredViewers.insert(rv);
	mutex.unlock();
}

set<RqCommon::RegisteredViewer> RqCommon::getAllViewers()
{
	mutex.lock(HERE);
	set<RegisteredViewer> r = registeredViewers;
	mutex.unlock();
	return r;
}

set<RqCommon::RegisteredViewer> RqCommon::getViewersForClass(const string& className)
{
	set<RegisteredViewer> r;
	mutex.lock(HERE);
	for (set<RegisteredViewer>::iterator it = registeredViewers.begin(); it != registeredViewers.end(); ++it) {
		if (it->targetClass == className) r.insert(*it);
	}
	mutex.unlock();
	return r;
}

void RqCommon::registerModuleWidget(const string& moduleName, QWidget* widget)
{
	mutex.lock(HERE);
	moduleWidgetMap.insert(make_pair(moduleName, widget));
	mutex.unlock();
}

void RqCommon::unregisterModuleWidget(const string& moduleName)
{
	mutex.lock(HERE);
	map<string, QWidget*>::iterator it = moduleWidgetMap.find(moduleName);
	if (it == moduleWidgetMap.end()) {
		RDK_ERROR_PRINTF("You want to unregister the widget of an unknown module '%s'", moduleName.c_str());
	}
	else moduleWidgetMap.erase(it);
	mutex.unlock();
}

QWidget* RqCommon::getModuleWidget(const string& moduleName)
{
	QWidget* w = 0;
	mutex.lock(HERE);
	map<string, QWidget*>::iterator it = moduleWidgetMap.find(moduleName);
	if (it != moduleWidgetMap.end()) w = it->second;
	mutex.unlock();
	return w;
}

vector<RqCommon::RegisteredTool> RqCommon::getAllTools()
{
	mutex.lock(HERE);
	vector<RegisteredTool> r = registeredTools;
	mutex.unlock();
	return r;
}

bool RqCommon::registerGuiSession(Session* session)
{
	if (guiSession) {
		RDK_ERROR_PRINTF("The guiSession has already been registered");
		return false;
	}
	guiSession = session;
	return true;
}

void RqCommon::createTreeWidget(const string& agentName)
{
	createWidget("treeviewer_" + agentName + "_", "rdkrqm_treeviewermodule", string() + "agentName=" + agentName,
		QPoint(100, 100), QSize(300, 300));
}

void RqCommon::createDefaultViewerFor(const string& propertyUrl, const QPoint& p, int parentId)
{
	string className = "";
	SESSION_TRY_START(guiSession)
	className = guiSession->getPropertyClassName(propertyUrl);
	SESSION_END_CATCH_TERMINATE(guiSession)
	if (className == "") return;

	string libraryName = "";
	mutex.lock(HERE);
	set<RegisteredViewer> r = getViewersForClass(className);
	for (set<RegisteredViewer>::iterator it = r.begin(); it != r.end(); ++it) {
		if (it->defaultViewer) { libraryName = it->libraryName; break; }
	}
	mutex.unlock();
	if (libraryName == "") return;

	createViewerFor(propertyUrl, libraryName, p, parentId);
}

void RqCommon::createViewerFor(const string& propertyUrl, const string& libraryName, const QPoint& p, int parentId)
{
	string prefix = libraryName.substr(7);			// strips the "rdkrqm_" part
	prefix = prefix.substr(0, prefix.size() - 6);	// strips the "module" part
	prefix = prefix + "_" + ((Url)propertyUrl).getRelativeUrl(((Url)propertyUrl).getLevel()-1) + "_";
	Url u = guiSession->getRepository()->normalizeUrl(propertyUrl);
	createWidget(prefix, libraryName, "url=" + u, p, QSize(0, 0), parentId);
}

void RqCommon::createWidget(const string& namePrefix, const string& libraryName, const string& textConfig,
	const QPoint& p, const QSize& s, int parentId)
{
	bool exiting = false;	// FIXME facciamolo meglio
	QT_THREAD_GUARD()
	string moduleName = namePrefix;
	for (int i = 0;; i++) {
		moduleName = namePrefix + RDK2::TextUtils::toString(i);
		if (!getModuleWidget(moduleName)) break;
	}

	ModuleConfig mc;
	mc.library = libraryName;
	mc.moduleName = moduleName;

	// FIXME duplicate code
	vector<string> configLines = RDK2::TextUtils::tokenize(textConfig, "\n");
	for (size_t i = 0; i < configLines.size(); i++) {
		vector<string> a = RDK2::TextUtils::tokenize(configLines[i], "=");
		if (a.size() != 2) {
			RDK_ERROR_PRINTF("Malformed configuration string (%s)", textConfig.c_str());
		}
		else {
			mc.objConfigs.push_back(new Pair(RDK2::TextUtils::trim(a[0]), new RString(RDK2::TextUtils::trim(a[1]))));
		}
	}

	moduleManager->instantiateAdditionalModule(mc);


	if (parentId != -1) {
		bool found = false;
		SESSION_TRY_START(guiSession)
		guiSession->lock(RqCommon::getGuiStateUrl(), HERE);
		RGuiState* gs = guiSession->getObjectAsL<RGuiState>(RqCommon::getGuiStateUrl());
		for (Vector<RGuiWindow>::iterator wi = gs->windows.begin(); wi != gs->windows.end(); ++wi) {
			RGuiWindow& w = *(*wi);
			if (w.id == parentId) {
				if (w.tabs.size() > 0) {
					w.tabs[0]->moduleNames.push_back(new RString(moduleName));
					w.resizeMe = true;
					found = true;
					break;
				}
			}

			for (Vector<RGuiTab>::iterator ti = w.tabs.begin(); ti != w.tabs.end(); ++ti) {
				RGuiTab& t = *(*ti);
				if (t.id == parentId) {
					t.moduleNames.push_back(new RString(moduleName));
					found = true;
					w.resizeMe = true;
					wi = gs->windows.end();	// exit from the outer for loop
					break;
				}
			}
		}
		guiSession->unlock(RqCommon::getGuiStateUrl());
		SESSION_END_CATCH_TERMINATE(guiSession)
		if (!found) parentId = -1;
		else {
			refreshGui();
		}
	}

	if (parentId == -1) {
		RGuiWindow* w = new RGuiWindow();
		if (libraryName == "rdkrqm_treeviewermodule") {
			if (namePrefix.size() > 12) {
				string agentName = namePrefix.substr(11, namePrefix.size() - 1 - 11); //guiSession->getRepositoryName();
				w->title = agentName + (agentName == guiSession->getRepositoryName() ? " (local repository)" : " (remote repository)");
			}
			else w->title = "ERROR";
		}
		else {
			w->title = "Viewer";
		}
		w->position.x = p.x();
		w->position.y = p.y();
		w->dimension.x = s.width();
		w->dimension.y = s.height();
		RGuiTab* t = new RGuiTab();
		if (libraryName == "rdkrqm_treeviewermodule") {
			t->title = "Tree";
		}
		else {
			t->title = "Tab";
		}
		t->moduleNames.push_back(new RString(moduleName));
		w->tabs.push_back(t);
		SESSION_TRY_START(guiSession)
				guiSession->lock(RqCommon::getGuiStateUrl(), HERE);
		RGuiState* gs = guiSession->getObjectAsL<RGuiState>(RqCommon::getGuiStateUrl());
		gs->windows.push_back(w);
		guiSession->unlock(RqCommon::getGuiStateUrl());
		SESSION_END_CATCH_TERMINATE(guiSession)
		refreshGui();
	}
}

void RqCommon::updateGuiState()
{
	bool exiting = false;
	QT_THREAD_GUARD()
	SESSION_TRY_START(guiSession)
	string guiStateUrl = getGuiStateUrl();
	guiSession->lock(guiStateUrl, HERE);
	RGuiState* gs = guiSession->getObjectAsL<RGuiState>(guiStateUrl);
	for (Vector<RGuiWindow>::iterator wi = gs->windows.begin(); wi != gs->windows.end(); ++wi) {
		RGuiWindow& w = *(*wi);
		if (w.qWindow) {
			QPoint position = w.qWindow->parentWidget()->pos();
			QSize size = w.qWindow->size();
			w.position = RDK2::RGeometry::RPoint2i(position.x(), position.y());
			w.dimension = RDK2::RGeometry::RPoint2i(size.width(), size.height());
		}
	}
	guiSession->unlock(guiStateUrl);
	SESSION_END_CATCH_TERMINATE(guiSession)
}

void RqCommon::refreshGui()
{
	bool exiting = false;
	QT_THREAD_GUARD()
	DEBUG_PRINTF("Refreshing GUI");
	qApp->processEvents();				// this is needed in order to make modules create their widgets
	set<string> alreadyBuiltWidgets;	// this will be used to avoid duplicated widgets for the same module
	SESSION_TRY_START(guiSession)
	guiSession->lock(RqCommon::getGuiStateUrl(), HERE);
	RGuiState* gs = guiSession->getObjectAsL<RGuiState>(RqCommon::getGuiStateUrl());
	for (Vector<RGuiWindow>::iterator wi = gs->windows.begin(); wi != gs->windows.end(); ) {
		RGuiWindow& w = *(*wi);

		if (!w.qWindow) {
			w.qWindow = new RqTabWidget(mainWindow->centralWidget(), w.title.c_str(), 0, *wi);
			w.qWindow->parent()->installEventFilter(windowEventFilter);	// this will filter paint events (I'm not able to filter move events)
			w.qWindow->installEventFilter(windowEventFilter);			// this will filter close events
			w.qWindow->setCaption(w.title);
			w.qWindow->move(QPoint(w.position.x, w.position.y));
			if (w.dimension.x == 0 || w.dimension.y == 0) w.qWindow->resize(w.qWindow->sizeHint());
			else w.qWindow->resize(QSize(w.dimension.x, w.dimension.y));
		}

		for (Vector<RGuiTab>::iterator ti = w.tabs.begin(); ti != w.tabs.end(); ) {
			RGuiTab& t = *(*ti);

			if (!t.qTab) {
				t.qTab = new QWidget(w.qWindow);
				new QVBoxLayout(t.qTab);
				w.qWindow->addTab(t.qTab, t.title);
				t.qTab->show();
			}

			bool someNewWidget = false;
			for (Vector<RString>::iterator mi = t.moduleNames.begin(); mi != t.moduleNames.end(); ) {
				string m = (*mi)->value;
				DEBUG_PRINTF("Want to build widget for '%s'", m.c_str());
				if (alreadyBuiltWidgets.find(m) != alreadyBuiltWidgets.end()) {
					RDK_ERROR_PRINTF("Found a duplicated widget in tab '%s' of window '%s', removing", t.title.c_str(), w.title.c_str());
					mi = t.moduleNames.erase(mi, true);
				}
				else {
					QWidget* wdg = getModuleWidget(m);
					if (!wdg) {
						DEBUG_PRINTF("No widget registered for module '%s', removing widget entry from tab", m.c_str());
						mi = t.moduleNames.erase(mi, true);
					}
					else {
						if (((QBoxLayout*)(t.qTab->layout()))->findWidget(wdg) == -1) {
							if (mi != t.moduleNames.begin()) ((QBoxLayout*)(t.qTab->layout()))->addStretch(1);
							wdg->reparent(t.qTab, QPoint(10, 10), true);
							((QBoxLayout*)(t.qTab->layout()))->addWidget(wdg);
							DEBUG_PRINTF("Showing widget '%s'", m.c_str());
							wdg->show();
							someNewWidget = true;
						}
						alreadyBuiltWidgets.insert(m);
						++mi;
					}
				}
			}
			someNewWidget = someNewWidget;

			if (t.moduleNames.size() == 0) {
				DEBUG_PRINTF("The tab '%s' of window '%s' is empty, removing", t.title.c_str(), w.title.c_str());
				t.qTab->deleteLater();
				ti = w.tabs.erase(ti, true);
			}
			else ++ti;
		}
		if (w.tabs.size() == 0) {
			DEBUG_PRINTF("The window '%s' is empty, removing", w.title.c_str());
			w.qWindow->deleteLater();
			wi = gs->windows.erase(wi, true);
		}
		else ++wi;
		if (w.resizeMe) {
			QSize s = w.qWindow->minimumSizeHint();
			w.qWindow->resize(QSize(std::max(s.width(), w.qWindow->width()), std::max(s.height(), w.qWindow->height())));
			w.resizeMe = false;
		}
		w.qWindow->show();
	}

	guiSession->unlock(RqCommon::getGuiStateUrl());
	SESSION_END_CATCH_TERMINATE(guiSession)
}

bool RcWindowEventFilter::eventFilter(QObject* obj, QEvent* e)
{
	if (e->type() == QEvent::Close) {
		Session* guiSession = RqCommon::getGuiSession("");
		SESSION_TRY_START(guiSession)
		guiSession->lock(RqCommon::getGuiStateUrl(), HERE);

		RGuiState* gs = guiSession->getObjectAsL<RGuiState>(RqCommon::getGuiStateUrl());
		RqTabWidget* tabw = dynamic_cast<RqTabWidget*>(obj);
		if (tabw) {
			for (Vector<RGuiWindow>::iterator wi = gs->windows.begin(); wi != gs->windows.end(); ++wi) {
				RGuiWindow& w = *(*wi);
				if (w.qWindow == tabw) {
					for (Vector<RGuiTab>::iterator ti = w.tabs.begin(); ti != w.tabs.end(); ++ti) {
						RGuiTab& t = *(*ti);
						for (Vector<RString>::iterator mi = t.moduleNames.begin(); mi != t.moduleNames.end(); ++mi) {
							string moduleName = (*mi)->value;
							DEBUG_PRINTF("I want to destroy module '%s'", moduleName.c_str());
							RqCommon::getModuleManager()->deleteModule(moduleName);
							// the widget will be destroyed by the module itself
						}
						t.moduleNames.clear(true);
					}
					break;
				}
			}
		}

		guiSession->unlock(RqCommon::getGuiStateUrl());
		SESSION_END_CATCH_TERMINATE(guiSession)
		RqCommon::refreshGui();		// this will delete also empty tabs and empty windows
		return true;	// block this event
	}
	else if (e->type() == QEvent::Paint) {
		RqCommon::updateGuiState();
		return false;
	}
	else return false;
}

}}
