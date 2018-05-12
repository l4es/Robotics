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

#include "queueviewermodule.h"

#include <qlabel.h>

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "QueueViewerModule"

#include <rdkcore/rprimitive/rint.h>

namespace RDK2 { namespace RConsoleQt {

QWidget* QueueViewerModule::createWidget()
{
	string myUrl = "<error>";
	SESSION_TRY_START(session)
		myUrl = session->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
		session->listenToTimer(1000.);
	SESSION_END_CATCH_TERMINATE(session)

	QWidget* widget = new QWidget();

	QVBoxLayout* mainLayout = new QVBoxLayout(widget);
	QLabel* label = new QLabel(SMALL_URL_CAPTION(myUrl.c_str()), widget);
	QWidget* editPanel = new QWidget(widget);
	editPanel->setPaletteForegroundColor(Qt::red);
	mainLayout->addWidget(label);
	mainLayout->addWidget(editPanel);

	QHBoxLayout* statLayout = new QHBoxLayout(mainLayout);
	statLayout->addWidget(lblStats = new QLabel(widget));
	editWidgets.push_back(lblStats);

	QHBoxLayout* cstatLayout = new QHBoxLayout(mainLayout);
	cstatLayout->addWidget(lblCurrent = new QLabel(widget));
	editWidgets.push_back(lblCurrent);

	return widget;
}

RDK2::Object* QueueViewerModule::buildRObject()
{
	return 0;
}

void QueueViewerModule::refreshWidgetFields()
{
	QT_THREAD_GUARD()
	//vector<const RDK2::Object*> objs;
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	string value;
	size_t size=0;
	SESSION_TRY_START(guiSession)
		string url = guiSession->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
		size = guiSession->queueSize(url);
		guiSession->queueFreeze(url);
	SESSION_END_CATCH_TERMINATE(guiSession)


	if (tmr.getSeconds() > 1.)
	{
		size_t diff = size - prevsize;
		lblStats->setText(QString("%1 elements/second in the queue").arg(diff));
		prevsize = size;
		tmr.start();
	}
	lblCurrent->setText(QString("%1 elements currently in the queue").arg(size));
}

MODULE_FACTORY(QueueViewerModule);

}} // ns
