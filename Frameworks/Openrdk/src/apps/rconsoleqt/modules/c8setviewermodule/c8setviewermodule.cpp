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

#include "c8setviewermodule.h"

#include <rdkcore/rgraphics/rc8set.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "C8SetViewerModule"

#include <qlabel.h>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RGraphics;

QWidget* C8SetViewerModule::createWidget()
{
	string myUrl = "<error>";
	SESSION_TRY_START(session)
		myUrl = session->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
	SESSION_END_CATCH_TERMINATE(session)
	
	QWidget* widget = new QWidget();
	
	QVBoxLayout* mainLayout = new QVBoxLayout(widget);
	QLabel* label = new QLabel(SMALL_URL_CAPTION(myUrl.c_str()), widget);
	QWidget* editPanel = new QWidget(widget);

	mainLayout->addWidget(label);
	mainLayout->addWidget(editPanel);

	QVBoxLayout* editLayout = new QVBoxLayout(mainLayout);
	editLayout->addWidget(cbWhite = new QCheckBox("White", widget));
	cbWhite->setPaletteBackgroundColor(Qt::white);
	editLayout->addWidget(cbBlack = new QCheckBox("Black", widget));
	cbBlack->setPaletteForegroundColor(Qt::black);
	editLayout->addWidget(cbBlue = new QCheckBox("Blue", widget));
	cbBlue->setPaletteForegroundColor(Qt::blue);
	editLayout->addWidget(cbRed = new QCheckBox("Red", widget));
	cbRed->setPaletteForegroundColor(Qt::red);
	editLayout->addWidget(cbGreen = new QCheckBox("Green", widget));
	cbGreen->setPaletteForegroundColor(Qt::green);
	editLayout->addWidget(cbMagenta = new QCheckBox("Magenta", widget));
	cbMagenta->setPaletteForegroundColor(Qt::magenta);
	editLayout->addWidget(cbCyan = new QCheckBox("Cyan", widget));
	cbCyan->setPaletteForegroundColor(Qt::cyan);
	editLayout->addWidget(cbGrey = new QCheckBox("Grey", widget));
	cbGrey->setPaletteBackgroundColor(Qt::gray);
	
	editWidgets.push_back(cbWhite);
	editWidgets.push_back(cbBlack);
	editWidgets.push_back(cbBlue);
	editWidgets.push_back(cbRed);
	editWidgets.push_back(cbGreen);
	editWidgets.push_back(cbMagenta);
	editWidgets.push_back(cbCyan);
	editWidgets.push_back(cbGrey);

	return widget;
}

RDK2::Object* C8SetViewerModule::buildRObject()
{
	RC8Set* c8set = new RC8Set();
	if (cbWhite->isChecked()) c8set->set(RImage::C8White);
	if (cbBlack->isChecked()) c8set->set(RImage::C8Black);
	if (cbBlue->isChecked()) c8set->set(RImage::C8Blue);
	if (cbRed->isChecked()) c8set->set(RImage::C8Red);
	if (cbGreen->isChecked()) c8set->set(RImage::C8Green);
	if (cbMagenta->isChecked()) c8set->set(RImage::C8Magenta);
	if (cbCyan->isChecked()) c8set->set(RImage::C8Cyan);
	if (cbGrey->isChecked()) c8set->set(RImage::C8Grey);
	return c8set;
}

void C8SetViewerModule::refreshWidgetFields()
{
	QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	RC8Set* c8set = 0;
	SESSION_TRY_START(guiSession)
		string url = guiSession->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
		guiSession->lock(url, HERE);
		RC8Set* objVal = guiSession->getObjectAsL<RC8Set>(url);
		if (objVal) {
			c8set = (RC8Set*) objVal->clone();
			received = true;
		}
		else {
			received = false;
		}
		guiSession->unlock(url);
	SESSION_END_CATCH_TERMINATE(guiSession)
	
	if (c8set) {
			cbWhite->setChecked(c8set->contains(RImage::C8White));
			cbBlack->setChecked(c8set->contains(RImage::C8Black));
			cbBlue->setChecked(c8set->contains(RImage::C8Blue));
			cbRed->setChecked(c8set->contains(RImage::C8Red));
			cbGreen->setChecked(c8set->contains(RImage::C8Green));
			cbMagenta->setChecked(c8set->contains(RImage::C8Magenta));
			cbCyan->setChecked(c8set->contains(RImage::C8Cyan));
			cbGrey->setChecked(c8set->contains(RImage::C8Grey));
			delete c8set;
	}
}
	
MODULE_FACTORY(C8SetViewerModule);

}} // ns
