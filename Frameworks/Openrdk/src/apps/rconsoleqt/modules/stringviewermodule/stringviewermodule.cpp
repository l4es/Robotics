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

#include "stringviewermodule.h"

#include <qlabel.h>

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "StringViewerModule"

#include <rdkcore/rprimitive/rstring.h>

namespace RDK2 { namespace RConsoleQt {
	
QWidget* StringViewerModule::createWidget()
{
	string myUrl = "<error>";
	SESSION_TRY_START(session)
		myUrl = session->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
	SESSION_END_CATCH_TERMINATE(session)
	
	QWidget* widget = new QWidget();
	
	QVBoxLayout* mainLayout = new QVBoxLayout(widget);
	QLabel* label = new QLabel(SMALL_URL_CAPTION(myUrl.c_str()), widget);
	QWidget* editPanel = new QWidget(widget);
	editPanel->setPaletteForegroundColor(Qt::red);
	mainLayout->addWidget(label);
	mainLayout->addWidget(editPanel);
	
	QHBoxLayout* editLayout = new QHBoxLayout(mainLayout);
	editLayout->addWidget(lineEdit = new QLineEdit(widget));
	
	editWidgets.push_back(lineEdit);
	
	return widget;
}

RDK2::Object* StringViewerModule::buildRObject()
{
	RDK2::Object* obj = new RString();
	if (obj->loadFromStringRepresentation(lineEdit->text().latin1())) {
		return obj;
	}
	else {
		delete obj;
		return 0;
	}
}

void StringViewerModule::refreshWidgetFields()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	string value;
	SESSION_TRY_START(guiSession)
		string url = guiSession->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
		guiSession->lock(url, HERE);
		RDK2::Object* objVal = guiSession->getObjectL(url);
		value = (objVal ? objVal->getStringRepresentation() : "<unset>");
		if (objVal) received = true;
		else received = false;
		guiSession->unlock(url);
	SESSION_END_CATCH_TERMINATE(guiSession)
	
	lineEdit->setText(QString(value));
}
	
MODULE_FACTORY(StringViewerModule);

}} // ns
