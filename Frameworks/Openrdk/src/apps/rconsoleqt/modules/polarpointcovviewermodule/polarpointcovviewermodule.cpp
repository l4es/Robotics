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

#include "polarpointcovviewermodule.h"

#include <rdkcore/geometry/angle.h>
#include <rdkcore/rgeometry/rpointstat.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "PolarPointCovViewerModule"

#include <qlabel.h>

namespace RDK2 { namespace RConsoleQt {

using namespace Geometry;

QWidget* PolarPointCovViewerModule::createWidget()
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
	editLayout->addWidget(new QLabel("Rho:", widget));
	editLayout->addWidget(leRho = new QLineEdit(widget));
	editLayout->addWidget(new QLabel("Theta (deg):", widget));
	editLayout->addWidget(leTheta = new QLineEdit(widget));

	editWidgets.push_back(leRho);
	editWidgets.push_back(leTheta);
	leRho->setMinimumSize(45,20);
	leTheta->setMinimumSize(45,20);

	QGridLayout* covLayout = new QGridLayout(editLayout, 2, 2);
	covLayout->addWidget(leST  = new QLineEdit(widget), 1, 1);
	covLayout->addWidget(leSTR = new QLineEdit(widget), 1, 2);
	covLayout->addWidget(leSRT = new QLineEdit(widget), 2, 1);
	covLayout->addWidget(leSR  = new QLineEdit(widget), 2, 2);
	leST->setMinimumSize(45,20);
	leSTR->setMinimumSize(45,20);
	leSRT->setMinimumSize(45,20);
	leSR->setMinimumSize(45,20);
	editWidgets.push_back(leST);
	editWidgets.push_back(leSTR);
	editWidgets.push_back(leSRT);
	editWidgets.push_back(leSR);

	return widget;
}

RDK2::Object* PolarPointCovViewerModule::buildRObject()
{
	RDK2::Object* obj = new RPolarPointCov(
			deg2rad(leTheta->text().toDouble()),leRho->text().toDouble(),
			leST->text().toDouble(), leSR->text().toDouble(), leSRT->text().toDouble()) ;
	return obj;
}

void PolarPointCovViewerModule::refreshWidgetFields()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	PolarPointCov value;
	bool unset = false;
	SESSION_TRY_START(guiSession)
		string url = guiSession->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
		guiSession->lock(url, HERE);
		RPolarPointCov* objVal = guiSession->getObjectAsL<RPolarPointCov>(url);
		if (objVal)
		{
			value.rho = objVal->rho;
			value.theta = objVal->theta;
			value.cov = objVal->cov;
		}
		else
		{
			unset = true;
		}
		guiSession->unlock(url);
	SESSION_END_CATCH_TERMINATE(guiSession)

	if (!unset) {
		QString qs;
		leRho->setText(qs.sprintf("%.3f", value.rho));
		leTheta->setText(qs.sprintf("%.3f", rad2deg(value.theta)));
		leST->setText(qs.sprintf("%.3f", value.cov.xx));
		leSTR->setText(qs.sprintf("%.3f", value.cov.xy));
		leSRT->setText(qs.sprintf("%.3f", value.cov.xy));
		leSR->setText(qs.sprintf("%.3f", value.cov.yy));
		received = true;
	}
	else {
		leRho->setText("<unset>");
		leTheta->setText("<unset>");
		leST->setText("<unset>");
		leSTR->setText("<unset>");
		leSRT->setText("<unset>");
		leSR->setText("<unset>");
		received = false;
	}
}

MODULE_FACTORY(PolarPointCovViewerModule);

}} // ns
