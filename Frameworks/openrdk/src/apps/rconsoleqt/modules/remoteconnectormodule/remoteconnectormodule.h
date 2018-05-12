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

#ifndef RDK2_RCW_REMOTECONNECTORMODULE
#define RDK2_RCW_REMOTECONNECTORMODULE

#include <qlineedit.h>
#include <qpushbutton.h>
#include <qlayout.h>
#include <qlistbox.h>

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RepositoryNS;
using namespace RDK2::RAgent;

class RemoteConnectorModule : public QObject, public Module {
Q_OBJECT
private:
	QLineEdit* tbConnectTo;
	QPushButton* btnCancel;
	QPushButton* btnConnect;
	QListBox* lbYpHosts;

	QWidget* createWidget();
	volatile bool widgetReady;
	void customEvent(QCustomEvent* e);

public slots:
	void btnConnect_clicked();
	void btnCancel_clicked();
	void lbYpHosts_clicked(QListBoxItem* lbi);

public:
	RemoteConnectorModule() : widgetReady(false) { }
	bool initConfigurationProperties();
	bool init();
	void exec();
};

}} // ns

#endif
