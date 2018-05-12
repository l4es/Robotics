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

#include "bigpropertyviewermodule.h"
#include "rqcommon.h"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "BigPropertyViewerModule"

#include <qcombobox.h>
#include <qlineedit.h>
#include <qcheckbox.h>
#include <qapplication.h>
#include <qtooltip.h>
#include <qmenubar.h>
#include <qtimer.h>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RAgent;

bool BigPropertyViewerModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
	RDK2::Common::createDefaultProperties(session, true, false);
	session->createString(PROPERTY_URL, "(do not use this)", "");
	session->createBool(PROPERTY_SHOW_SIDE_WIDGET, "Show side widget", true);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool BigPropertyViewerModule::init()
{
	QApplication::postEvent(this, new QCustomEvent(RqCommon::EVENT_CREATE_WIDGET));
	return true;
}

void BigPropertyViewerModule::cleanup()
{
	QWidget* widget = RqCommon::getModuleWidget(getModuleName());
	RqCommon::unregisterModuleWidget(getModuleName());
	widget->deleteLater();		// XXX I'm not sure if this is safe to be called outside the QT thread
}

void BigPropertyViewerModule::customEvent(QCustomEvent* e)
{
	if (e->type() == RqCommon::EVENT_CREATE_WIDGET) {
		RqCommon::registerModuleWidget(getModuleName(), createWholeWidget());
		widgetReady = true;
	}
}

void BigPropertyViewerModule::slot_refresh()
{
	refreshViewer();
}

void BigPropertyViewerModule::slot_chkShowSideWidget_toggled(bool t)
{
	if (t) sideWidget->show();
	else sideWidget->hide();
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
	guiSession->setBool(PROPERTY_SHOW_SIDE_WIDGET, t);
	SESSION_END_CATCH_TERMINATE(guiSession)
}

QWidget* BigPropertyViewerModule::createWholeWidget()
{
	QT_THREAD_GUARD(0)
	
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	bool sidePanelIsShown = true;
	SESSION_TRY_START(guiSession)
	sidePanelIsShown = guiSession->getBool(PROPERTY_SHOW_SIDE_WIDGET);
	SESSION_END_CATCH_TERMINATE(guiSession)
	
	QWidget* mainWidget = new QWidget();
		// these are created before side widget and viewer widget, so you have these objects in those functions
		chkShowSideWidget = new QCheckBox("Show side panel", mainWidget);
			chkShowSideWidget->setChecked(sidePanelIsShown);
			QObject::connect(chkShowSideWidget, SIGNAL(toggled(bool)),
				this, SLOT(slot_chkShowSideWidget_toggled(bool)));
		lblStatus = new QLabel("", mainWidget);
			lblStatus->setAlignment(Qt::AlignRight);
	
	QVBoxLayout* mainLayout = new QVBoxLayout(mainWidget, 0);
		QHBoxLayout* mainPanelLayout = new QHBoxLayout(mainLayout, 10);
			sideWidget = createSideWidget(mainWidget);
				sideWidget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
				if (!sidePanelIsShown) sideWidget->hide();
			viewerWidget = createViewerWidget(mainWidget);
				viewerWidget->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
				viewerWidget->setMinimumSize(200, 0);         // FIXME
			mainPanelLayout->addWidget(sideWidget);
			mainPanelLayout->addWidget(viewerWidget);
			viewerWidget->show();

		QHBoxLayout* bottomLayout = new QHBoxLayout(mainLayout);
			bottomLayout->addWidget(chkShowSideWidget);
			bottomLayout->addWidget(lblStatus);

	mainWidget->installEventFilter(this);
	
	QObject::connect(&refreshTimer, SIGNAL(timeout()), this, SLOT(slot_refresh()));
	
	return mainWidget;
}

void BigPropertyViewerModule::exec()
{
	WIDGET_GUARD
	
	while (session->wait(), !exiting) { }
}

bool BigPropertyViewerModule::eventFilter(QObject* /*obj*/, QEvent* e)
{
	if (e->type() == QEvent::MouseButtonRelease) {
		QMouseEvent* me = (QMouseEvent*)e;
		if (me->button() == QMouseEvent::RightButton) {
			QPopupMenu* m = new QPopupMenu();
			QObject::connect(m, SIGNAL(activated(int)), this, SLOT(slot_popupActivated(int)));
			m->insertItem("Close this viewer", 0);
			m->popup(me->globalPos());
			return true;
		}
		else return false;
	}
	else return false;
}
		
void BigPropertyViewerModule::slot_popupActivated(int a)
{
	QT_THREAD_GUARD()
	if (a == 0) {
		getModuleManager()->deleteModule(getModuleName());	// suicide!
		RqCommon::refreshGui();
	}
}

}}
