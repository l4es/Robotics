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
#include <qmenubar.h>
#include <qdragobject.h>
#include <qapplication.h>

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RqMainWindow"

#include "rqmainwindow.h"
#include "rqcommon.h"

namespace RDK2 { namespace RConsoleQt {

RqMainWindow::RqMainWindow()
{
	createMenuBar();
	workspace = new RqMainWindowWorkspace(this);
	workspace->setScrollBarsEnabled(true);
	setCentralWidget(workspace);
}

RqMainWindow::~RqMainWindow()
{ }

void RqMainWindow::slot_showMyRepository()
{
	Session* guiSession = RqCommon::getGuiSession("");
	RqCommon::createTreeWidget(guiSession->getRepositoryName());
}

void RqMainWindow::slot_connectToRemoteRepository()
{
	RqCommon::createWidget("treeviewer_", "rdkrqm_remoteconnectormodule", "", QPoint(100, 100), QSize(100, 100));
}

void RqMainWindow::createMenuBar()
{
	QMenuBar* bar = menuBar();
	
	QPopupMenu* networkMenu = new QPopupMenu(this, "FileMenu");
	networkMenu->connectItem(networkMenu->insertItem("&Show my repository"),
		this, SLOT(slot_showMyRepository()));
	networkMenu->connectItem(networkMenu->insertItem("&Connect to remote repository..."),
		this, SLOT(slot_connectToRemoteRepository()));
	networkMenu->insertSeparator();
	networkMenu->connectItem(networkMenu->insertItem("&Quit"),
		qApp, SLOT(quit()));
	bar->insertItem("&Network", networkMenu);
	
	QPopupMenu* toolsMenu = new QPopupMenu( this, "ToolsMenu" );
	vector<RqCommon::RegisteredTool> r = RqCommon::getAllTools();
	int i = 0;
	for (vector<RqCommon::RegisteredTool>::iterator it = r.begin(); it != r.end(); ++it) {
		if (it->menuCaption == "-") { toolsMenu->insertSeparator(); i++; }
		else toolsMenu->connectItem(toolsMenu->insertItem(it->menuCaption, i++), this, SLOT(slot_toolMenuClicked(int)));
	}
	bar->insertItem("&Tools", toolsMenu);
}

void RqMainWindow::slot_toolMenuClicked(int id)
{
	bool exiting = false;
	QT_THREAD_GUARD()
	vector<RqCommon::RegisteredTool> r = RqCommon::getAllTools();
	int i = 0;
	for (vector<RqCommon::RegisteredTool>::iterator it = r.begin(); it != r.end(); ++it) {
		if (i == id) {
			RqCommon::createWidget("tool_", it->libraryName, "", QPoint(100, 100), QSize(100, 100));
			return;
		}
		i++;
	}
}

void RqMainWindowWorkspace::dragEnterEvent(QDragEnterEvent* event)
{
	event->accept(QTextDrag::canDecode(event));
}

void RqMainWindowWorkspace::dropEvent(QDropEvent* event)
{
	QString qsUrl;
	if (QTextDrag::decode(event, qsUrl)) {
		Url u = qsUrl.latin1();
		RqCommon::createDefaultViewerFor(u, event->pos());
	}
}

}}
