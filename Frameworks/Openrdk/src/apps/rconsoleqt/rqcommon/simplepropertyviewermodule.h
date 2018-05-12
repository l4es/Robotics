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

#ifndef RDK2_RQ_SIMPLEPROPERTYVIEWERMODULE
#define RDK2_RQ_SIMPLEPROPERTYVIEWERMODULE

#include <qframe.h>
#include <qlayout.h>
#include <qlineedit.h>
#include <qpushbutton.h>

#include <rdkcore/repository/session.h>
#include <rdkcore/modules/module.h>

#include "rqcommon.h"

#define PROPERTY_SIMPLEPROPERTYVIEWER_URL "url"

// these are colors for editable properties
#define QCOLOR_ERROR Qt::red
#define QCOLOR_EDITING QColor(255, 255, 150)
#define QCOLOR_SENT Qt::yellow
#define QCOLOR_RECEIVED QColor(200, 255, 200)
#define QCOLOR_UNSET Qt::gray

// these are colors for readonly properties
#define QCOLOR_RECEIVED_RO Qt::white
#define QCOLOR_UNSET_RO Qt::darkGray

#define SMALL_URL_CAPTION(caption) QString("<small>")+QString(caption)+QString("</small>")

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RepositoryNS;
using namespace RDK2::RAgent;

class SimplePropertyViewerModule : public QObject, public Module {
Q_OBJECT
public:
	// *structors
	SimplePropertyViewerModule() : received(false), readonly(false), editing(false), widgetReady(false) { }
	virtual ~SimplePropertyViewerModule() { }
	
	// module interface
    virtual bool initConfigurationProperties();
	virtual bool init();
	virtual void exec();
	virtual void cleanup();

protected:
	static const int EVENT_REFRESH = (RqCommon::EVENT_LOCAL + 1);

	virtual void refreshColor();
	virtual void setColor(const QColor& color);
	virtual bool eventFilter(QObject* obj, QEvent* e);
	virtual void customEvent(QCustomEvent* e);

	virtual QWidget* createWholeWidget();

public slots:
	virtual void slot_changing();
	virtual void slot_changed();
	virtual void slot_popupActivated(int a);

protected:
	/// all these functions will be called from the QT thread
	virtual QWidget* createWidget() = 0;
	virtual void refreshWidgetFields() = 0;		// you have to set received to whether true or false in this function
	virtual RDK2::Object* buildRObject() = 0;

	/// this function is called from the module thread
	/// you CAN reimplement it if you need to listen to different
	/// properties
	virtual bool initListening();

	vector<QWidget*> editWidgets;
	volatile bool received, readonly;
	volatile bool editing;
	volatile bool widgetReady;
};

}}

#endif
