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

#include "simplepropertyviewermodule.h"
#include "rqcommon.h"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "SimplePropertyViewerModule"

#include <qcombobox.h>
#include <qlineedit.h>
#include <qtextedit.h>
#include <qcheckbox.h>
#include <qapplication.h>
#include <qtooltip.h>
#include <qmenubar.h>
#include <qlabel.h>

namespace RDK2 { namespace RConsoleQt {

bool SimplePropertyViewerModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		RDK2::Common::createDefaultProperties(session, true, false);
		session->createString(PROPERTY_SIMPLEPROPERTYVIEWER_URL, "Url to display");
	SESSION_END_CATCH_TERMINATE(session)
	return true;
}

bool SimplePropertyViewerModule::init()
{
	QApplication::postEvent(this, new QCustomEvent(RqCommon::EVENT_CREATE_WIDGET));

	return initListening();
}

void SimplePropertyViewerModule::cleanup()
{
	QWidget* widget = RqCommon::getModuleWidget(getModuleName());
	RqCommon::unregisterModuleWidget(getModuleName());
	widget->deleteLater();		// XXX I'm not sure if this is safe to be called outside the QT thread
}

bool SimplePropertyViewerModule::initListening()
{
	SESSION_TRY_START(session)
		string url = session->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
		session->storageSubscribeValue(url);
		session->listen(url);
		session->wakeUp();
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return true;
}

void SimplePropertyViewerModule::customEvent(QCustomEvent* e)
{
	if (e->type() == RqCommon::EVENT_CREATE_WIDGET) {
		RqCommon::registerModuleWidget(getModuleName(), createWholeWidget());
		widgetReady = true;
	}
	else if (e->type() == EVENT_REFRESH) {
		if (!editing) {
			refreshWidgetFields();
			refreshColor();
		}
	}
}

void SimplePropertyViewerModule::refreshColor()
{
	if (!readonly) {
		if (editing) setColor(QCOLOR_EDITING);
		else if (received) setColor(QCOLOR_RECEIVED);
		else setColor(QCOLOR_UNSET);
	}
	else {
		if (received) setColor(QCOLOR_RECEIVED_RO);
		else setColor(QCOLOR_UNSET_RO);
	}
}

QWidget* SimplePropertyViewerModule::createWholeWidget()
{
	QT_THREAD_GUARD(0)

	QWidget* mainWidget = createWidget();

	mainWidget->installEventFilter(this);

	string propertyDesc = "";
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
			propertyDesc = guiSession->getPropertyDescription(guiSession->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL));
			readonly = guiSession->isReadonly(guiSession->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL));
	SESSION_END_CATCH_TERMINATE(guiSession)
	QToolTip::add(mainWidget, QString((propertyDesc + (readonly ? " (read only)" : "")).c_str()));

	for (size_t i = 0; i < editWidgets.size(); i++) {
		QLineEdit* le = dynamic_cast<QLineEdit*>(editWidgets[i]);
		if (le) {
			le->setAcceptDrops(false);
			connect(le, SIGNAL(textChanged(const QString&)), this, SLOT(slot_changing()));
			connect(le, SIGNAL(returnPressed()),             this, SLOT(slot_changed()));
			continue;
		}
		QCheckBox* cb = dynamic_cast<QCheckBox*>(editWidgets[i]);
		if (cb) {
			connect(cb, SIGNAL(clicked()), this, SLOT(slot_changed()));
			continue;
		}
		QComboBox* cbo = dynamic_cast<QComboBox*>(editWidgets[i]);
		if (cbo) {
			connect(cbo, SIGNAL(activated(int)), this, SLOT(slot_changed()));
			continue;
		}
		QLabel* lbl = dynamic_cast<QLabel*>(editWidgets[i]);
		if (lbl)
		{
			continue;
		}
		QTextEdit* te = dynamic_cast<QTextEdit*>(editWidgets[i]);
		if (te)
		{
			continue;
		}
		RDK_ERROR_PRINTF("Unknown editor class (edit widget index = %d)", i);
	}

	if (readonly) {
		for (size_t i = 0; i < editWidgets.size(); i++) {
			QLineEdit* le = dynamic_cast<QLineEdit*>(editWidgets[i]);
			if (le) le->setReadOnly(true);
			else editWidgets[i]->setEnabled(false);
		}
	}

	refreshColor();

	return mainWidget;
}

void SimplePropertyViewerModule::exec()
{
	WIDGET_GUARD

	while (session->wait(), !exiting) {
		QApplication::postEvent(this, new QCustomEvent(EVENT_REFRESH));
	}
}

void SimplePropertyViewerModule::setColor(const QColor& color)
{
	for (size_t i = 0; i < editWidgets.size(); i++) {
		QCheckBox* cb = dynamic_cast<QCheckBox*>(editWidgets[i]);
		if (cb) {
			QPalette p = cb->palette();
			p.setColor(QColorGroup::Base, color);
			cb->setPalette(p);
		}
		else {
			editWidgets[i]->setPaletteBackgroundColor(color);
		}
	}
}

bool SimplePropertyViewerModule::eventFilter(QObject* /*obj*/, QEvent* e)
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

void SimplePropertyViewerModule::slot_popupActivated(int a)
{
	QT_THREAD_GUARD()
	if (a == 0) {
		getModuleManager()->deleteModule(getModuleName());	// suicide!
		RqCommon::refreshGui();
	}
}

void SimplePropertyViewerModule::slot_changing()
{
	QT_THREAD_GUARD()
	editing = false;
	for (size_t i = 0; i < editWidgets.size(); i++) {
		QLineEdit* le = dynamic_cast<QLineEdit*>(editWidgets[i]);
		if (le && le->isModified()) editing = true;
	}
	refreshColor();
}

void SimplePropertyViewerModule::slot_changed()
{
	QT_THREAD_GUARD()
	RDK2::Object* value = buildRObject();

	if (!value) {
			RDK_ERROR_STREAM("Could not parse.");
			return;
	}

	setColor(QCOLOR_SENT);
	editing = false;
	for (size_t i = 0; i < editWidgets.size(); i++) {
		QLineEdit* le = dynamic_cast<QLineEdit*>(editWidgets[i]);
		if (le) le->clearModified();
	}

	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		string url = guiSession->getString(PROPERTY_SIMPLEPROPERTYVIEWER_URL);
		guiSession->setObject(url, value);
	SESSION_END_CATCH_TERMINATE(guiSession)
}

}}
