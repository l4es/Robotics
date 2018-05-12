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

#ifndef RDK2_RQM_GUI_MANAGER_MODULE
#define RDK2_RQM_GUI_MANAGER_MODULE

#include <set>
#include <string>

#include <qapplication.h>
#include <rdkcore/modules/module.h>
#include <rdkcore/posixconstructs/posixmutex.h>

#include "../../rqcommon/rqmainwindow.h"

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RAgent;
using namespace std;

class GuiManagerModule: public QObject, public Module {
public:
	bool initConfigurationProperties();
	bool init();
	void exec();
	void exitRequested();

private:
	QApplication* app;
	RqMainWindow* mainWindow;
};

}} // namespace

#endif
