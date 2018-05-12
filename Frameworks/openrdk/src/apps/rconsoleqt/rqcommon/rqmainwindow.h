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

#ifndef RDK2_RCONSOLE_QT_RCMAINWINDOW
#define RDK2_RCONSOLE_QT_RCMAINWINDOW

#include <qmainwindow.h>
#include <qworkspace.h>

#include <rdkcore/object/object.h>
#include <rdkcore/repository/session.h>
#include <rdkcore/repository/repository.h>
#include <rdkcore/posixconstructs/posixmutex.h>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RepositoryNS;

class RqMainWindowWorkspace;

/** 
 * This is the \p RConsoleQt's Main Window, it is created internally by
 * the \p rdk2rqm_guimanagermodule
 */
class RqMainWindow: public QMainWindow, public RDK2::Object {
Q_OBJECT
public:
	RqMainWindow();
	~RqMainWindow();

protected:
	void createMenuBar();

public:
	RqMainWindowWorkspace* workspace;

protected slots:
	void slot_showMyRepository();
	void slot_connectToRemoteRepository();
	void slot_toolMenuClicked(int);
};

class RqMainWindowWorkspace : public QWorkspace {
Q_OBJECT
public:
	RqMainWindowWorkspace(RqMainWindow* parent) : QWorkspace(parent) {
		setAcceptDrops(true);
	}

	PosixConstructs::PosixMutex mutex;
	void dragEnterEvent(QDragEnterEvent* event);
	void dropEvent(QDropEvent* event);

friend class RqMainWindow;
};

}} // ns

#endif
