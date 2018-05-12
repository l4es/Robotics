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

#include "rqmodule.h"
#include "rqcommon.h"

#include <qapplication.h>

namespace RDK2 { namespace RConsoleQt {

bool RqModule::init()
{
	QApplication::postEvent(this, new QCustomEvent(RqCommon::EVENT_CREATE_WIDGET));
	return true;
}

void RqModule::cleanup()
{
	QWidget* widget = RqCommon::getModuleWidget(getModuleName());
	RqCommon::unregisterModuleWidget(getModuleName());
	widget->deleteLater();		// XXX I'm not sure if this is safe to be called outside the QT thread
}

void RqModule::exec()
{
	WIDGET_GUARD
	
	while (session->wait(), !exiting) { }
}

void RqModule::customEvent(QCustomEvent* e)
{
	if (e->type() == RqCommon::EVENT_CREATE_WIDGET) {
		RqCommon::registerModuleWidget(getModuleName(), createWidget());
		widgetReady = true;
	}
}

}}
