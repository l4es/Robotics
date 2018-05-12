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

#include "filenamechoosermodule.h"

#include <qlabel.h>
#include <qfiledialog.h>

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "FilenameChooserModule"

#include <rdkcore/rprimitive/rstring.h>

namespace RDK2 { namespace RConsoleQt {

QWidget* FilenameChooserModule::createWidget()
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
	editLayout->addWidget(openB = new QPushButton("...", widget));

	connect( openB, SIGNAL( clicked() ), this, SLOT( slot_changed() ) );
	editWidgets.push_back(lineEdit);

	return widget;
}

RDK2::Object* FilenameChooserModule::buildRObject()
{
	RDK2::Object* obj = new RString();
	if (obj->loadFromStringRepresentation(lineEdit->text().latin1()))
	{
		return obj;
	}
	else
	{
		delete obj;
		return 0;
	}
}

void FilenameChooserModule::slot_changed()
{
	QT_THREAD_GUARD()
	QString value = QFileDialog::getOpenFileName(
										QString::null,
										"*.*",
										0,
										"open file dialog",
										"Choose a file",
										false);
	if (!value.isNull())
	{
		RDK2::Object* obj = new RString();
		if (obj->loadFromStringRepresentation(value.latin1()))
		{
			lineEdit->setText(value);

			Session* guiSession = RqCommon::getGuiSession(getModuleName());
			SESSION_TRY_START(guiSession)
				string url = guiSession->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
				guiSession->setObject(url, obj);
				guiSession->valueChanged(url);
			SESSION_END_CATCH_TERMINATE(guiSession)
		}
	}
}

void FilenameChooserModule::refreshWidgetFields()
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

MODULE_FACTORY(FilenameChooserModule);

}} // ns
