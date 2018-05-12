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

#include "point2odcovviewermodule.h"

#include <rdkcore/geometry/angle.h>
#include <rdkcore/rgeometry/rpointstat.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "Point2odCovViewerModule"

#include <qlabel.h>

namespace RDK2 { namespace RConsoleQt {

using namespace Geometry;

QWidget* Point2odCovViewerModule::createWidget()
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

	QGridLayout* covLayout = new QGridLayout(editLayout, 3, 3);
	covLayout->addWidget(leSXX  = new QLineEdit(widget), 1, 1);
	covLayout->addWidget(leSXY = new QLineEdit(widget), 1, 2);
	covLayout->addWidget(leSXT = new QLineEdit(widget), 1, 3);
	covLayout->addWidget(leSYX = new QLineEdit(widget), 2, 1);
	covLayout->addWidget(leSYY  = new QLineEdit(widget), 2, 2);
	covLayout->addWidget(leSYT  = new QLineEdit(widget), 2, 3);
	covLayout->addWidget(leSTX = new QLineEdit(widget), 3, 1);
	covLayout->addWidget(leSTY  = new QLineEdit(widget), 3, 2);
	covLayout->addWidget(leSTT  = new QLineEdit(widget), 3, 3);

	editWidgets.push_back(leSXX);
	editWidgets.push_back(leSXY);
	editWidgets.push_back(leSXT);
	editWidgets.push_back(leSYX);
	editWidgets.push_back(leSYY);
	editWidgets.push_back(leSYT);
	editWidgets.push_back(leSTX);
	editWidgets.push_back(leSTY);
	editWidgets.push_back(leSTT);

	return widget;
}

RDK2::Object* Point2odCovViewerModule::buildRObject()
{
	RDK2::Object* obj = new RPoint2odCov(
			leX->text().toDouble(),leY->text().toDouble(),deg2rad(leTheta->text().toDouble()),
			leSXX->text().toDouble(), leSYY->text().toDouble(), leSXY->text().toDouble(),
			leSTT->text().toDouble(), leSXT->text().toDouble(), leSYT->text().toDouble()
			) ;
	return obj;
}

void Point2odCovViewerModule::refreshWidgetFields()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	Point2odCov value;
	bool unset = false;
	SESSION_TRY_START(guiSession)
		string url = guiSession->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
		guiSession->lock(url, HERE);
		RPoint2odCov* objVal = guiSession->getObjectAsL<RPoint2odCov>(url);
		if (objVal)
		{
			value.x = objVal->x;
			value.y = objVal->y;
			value.theta = objVal->theta;
			value.cov = objVal->cov;
		}
		else unset = true;
		guiSession->unlock(url);
	SESSION_END_CATCH_TERMINATE(guiSession)

	if (!unset) {
		QString qs;
		leX->setText(qs.sprintf("%.3f", value.y));
		leY->setText(qs.sprintf("%.3f", value.y));
		leTheta->setText(qs.sprintf("%.3f", rad2deg(value.y)));
		leSXX->setText(qs.sprintf("%.3f", value.cov.xx));
		leSXY->setText(qs.sprintf("%.3f", value.cov.xy));
		leSXT->setText(qs.sprintf("%.3f", value.cov.xt));
		leSYX->setText(qs.sprintf("%.3f", value.cov.xy));
		leSYY->setText(qs.sprintf("%.3f", value.cov.yy));
		leSYT->setText(qs.sprintf("%.3f", value.cov.yt));
		leSTX->setText(qs.sprintf("%.3f", value.cov.xt));
		leSTY->setText(qs.sprintf("%.3f", value.cov.yt));
		leSTT->setText(qs.sprintf("%.3f", value.cov.tt));
		received = true;
	}
	else {
		leX->setText("<unset>");
		leY->setText("<unset>");
		leSXX->setText("<unset>");
		leSXY->setText("<unset>");
		leSXT->setText("<unset>");
		leSYX->setText("<unset>");
		leSYY->setText("<unset>");
		leSYT->setText("<unset>");
		leSTX->setText("<unset>");
		leSTY->setText("<unset>");
		leSTT->setText("<unset>");
		received = false;
	}
}

MODULE_FACTORY(Point2odCovViewerModule);

}} // ns
