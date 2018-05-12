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

#include "poseviewermodule.h"

#include <rdkcore/geometry/angle.h>
#include <rdkcore/rgeometry/rpoint2od.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "PoseViewerModule"

#include <qlabel.h>

namespace RDK2 { namespace RConsoleQt {

using namespace Geometry;

QWidget* PoseViewerModule::createWidget()
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
	editLayout->addWidget(new QLabel("X:", widget));
	editLayout->addWidget(leX = new QLineEdit(widget));
	editLayout->addWidget(new QLabel("Y:", widget));
	editLayout->addWidget(leY = new QLineEdit(widget));
	editLayout->addWidget(new QLabel("Theta:", widget));
	editLayout->addWidget(leTheta = new QLineEdit(widget));
	editLayout->addWidget(new QLabel("deg", widget));

	editWidgets.push_back(leX);
	editWidgets.push_back(leY);
	editWidgets.push_back(leTheta);

	return widget;
}

RDK2::Object* PoseViewerModule::buildRObject()
{
	RDK2::Object* obj = new RPoint2od(leX->text().toDouble(), leY->text().toDouble(), deg2rad(leTheta->text().toDouble()));
	return obj;
}

void PoseViewerModule::refreshWidgetFields()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	Point2od value;
	bool unset = false;
	SESSION_TRY_START(guiSession)
		string url = guiSession->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
		guiSession->lock(url, HERE);
		RPoint2od* objVal = guiSession->getObjectAsL<RPoint2od>(url);
		if (objVal) {
			value.x = objVal->x;
			value.y = objVal->y;
			value.theta = objVal->theta;
		}
		else unset = true;
		guiSession->unlock(url);
	SESSION_END_CATCH_TERMINATE(guiSession)

	if (!unset) {
		QString qs;
		leX->setText(qs.sprintf("%.4f", value.x));
		leY->setText(qs.sprintf("%.4f", value.y));
		leTheta->setText(qs.sprintf("%.3f", rad2deg(value.theta)));
		received = true;
	}
	else {
		leX->setText("<unset>");
		leY->setText("<unset>");
		leTheta->setText("<unset>");
		received = false;
	}
}

MODULE_FACTORY(PoseViewerModule);

}} // ns
