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

#ifndef RDK2_RQ_BIGPROPERTYVIEWERMODULE
#define RDK2_RQ_BIGPROPERTYVIEWERMODULE

#include "rqcommon.h"

#include <rdkcore/repository/session.h>
#include <rdkcore/modules/module.h>

#include <qframe.h>
#include <qlayout.h>
#include <qlabel.h>
#include <qtimer.h>
#include <qcheckbox.h>

#define PROPERTY_URL "url"
#define PROPERTY_SHOW_SIDE_WIDGET "params/showSideWidget"

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RepositoryNS;
using namespace RDK2::RAgent;

class BigPropertyViewerModule : public QObject, public Module {
Q_OBJECT
public:
	// *structors
	BigPropertyViewerModule() : refreshTimer(this), sideWidget(0), viewerWidget(0), widgetReady(false) { }
	virtual ~BigPropertyViewerModule() { }
	
	// module interface
    virtual bool initConfigurationProperties();
	virtual bool init();
	virtual void exec();
	virtual void cleanup();

protected:
	QTimer refreshTimer;

	virtual bool eventFilter(QObject* obj, QEvent* e);
	virtual void customEvent(QCustomEvent* e);

	virtual QWidget* createWholeWidget();

public slots:
	virtual void slot_popupActivated(int a);
	virtual void slot_refresh();
	virtual void slot_chkShowSideWidget_toggled(bool);
	
protected:
	/// all these functions will be called from the QT thread
	virtual QWidget* createViewerWidget(QWidget* parent) = 0;
	virtual QWidget* createSideWidget(QWidget* parent) = 0;
	virtual void refreshViewer() = 0;

	QWidget* sideWidget;
	QWidget* viewerWidget;
	
	QCheckBox* chkShowSideWidget;
	QLabel* lblStatus;

	volatile bool widgetReady;
};

}}

#endif
