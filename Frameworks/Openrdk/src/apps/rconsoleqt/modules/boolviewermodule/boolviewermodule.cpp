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

#include "boolviewermodule.h"

#include <rdkcore/rprimitive/rint.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "BoolViewerModule"

#include <qlabel.h>

namespace RDK2 { namespace RConsoleQt {
	
QWidget* BoolViewerModule::createWidget()
{
	string myUrl = "<error>";
	SESSION_TRY_START(session)
		myUrl = session->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
	SESSION_END_CATCH_TERMINATE(session)
	
	QWidget* widget = new QWidget();
	
	QHBoxLayout* editLayout = new QHBoxLayout(widget);
	editLayout->addWidget(new QLabel(SMALL_URL_CAPTION(myUrl.c_str()), widget));
	editLayout->addWidget(cb = new QCheckBox(widget));
	
	editWidgets.push_back(cb);
	
	return widget;
}

RDK2::Object* BoolViewerModule::buildRObject()
{
	return new RBool(cb->isChecked());
}

void BoolViewerModule::refreshWidgetFields()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	bool value = false;
	SESSION_TRY_START(guiSession)
		string url = guiSession->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
		guiSession->lock(url, HERE);
		RBool* objVal = guiSession->getObjectAsL<RBool>(url);
		if (objVal) {
			value = objVal->value;
			received = true;
		}
		else received = false;
		guiSession->unlock(url);
	SESSION_END_CATCH_TERMINATE(guiSession)
	
	cb->setChecked(value);
}
	
MODULE_FACTORY(BoolViewerModule);

}} // ns
