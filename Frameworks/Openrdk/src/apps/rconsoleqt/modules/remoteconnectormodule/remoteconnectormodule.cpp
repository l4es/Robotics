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

#include "remoteconnectormodule.h"
#include "../../rqcommon/rqcommon.h"

#include <rdkcore/rnetobjects/ryellowpages.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RemoteConnectorModule"

#include <qlabel.h>
#include <qlistbox.h>
#include <qregexp.h>
#include <qapplication.h>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RNetObjects;

bool RemoteConnectorModule::initConfigurationProperties()
{
	return true;
}

bool RemoteConnectorModule::init()
{
	QApplication::postEvent(this, new QCustomEvent(RqCommon::EVENT_CREATE_WIDGET));
	return true;
/*	SESSION_TRY_START(session)
	// we prepare to receive the answer
	session->queueSubscribe(PROPERTY_INBOX);
	session->queueSetInterests(PROPERTY_INBOX, new MyInterests(session->getString("agentName")));
	session->listen(PROPERTY_INBOX);
	if (session->getString("agentName") == session->getRepositoryName() || session->getString("agentName") == "") {
		// this is a local tree viewer
		session->listenToTreeChanges("/");
	}
	else {
		// this is a remote tree viewer
		session->listenToTreeChanges("rdk://" + session->getString("agentName") + "/");
	}
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return true;*/
}

void RemoteConnectorModule::customEvent(QCustomEvent* e)
{
	if (e->type() == RqCommon::EVENT_CREATE_WIDGET) {
		RqCommon::registerModuleWidget(getModuleName(), createWidget());
		widgetReady = true;
	}
}

QWidget* RemoteConnectorModule::createWidget()
{
	QT_THREAD_GUARD(0)
	
	QWidget* moduleWidget = new QWidget();
	QVBoxLayout* vlayout = new QVBoxLayout(moduleWidget, 10);
	vlayout->addWidget(new QLabel("Yellow pages:", moduleWidget));
	vlayout->addWidget(lbYpHosts = new QListBox(moduleWidget));
	QHBoxLayout* ll = new QHBoxLayout(vlayout);
	ll->addWidget(new QLabel("Connect to:", moduleWidget));
	ll->addWidget(tbConnectTo = new QLineEdit(moduleWidget, "ConnectTo"));
	ll = new QHBoxLayout(vlayout);
	ll->addWidget(btnConnect = new QPushButton("Connect", moduleWidget, "Connect"));
	ll->addWidget(btnCancel = new QPushButton("Cancel", moduleWidget, "Cancel"));
	moduleWidget->setMinimumSize(300, 50);
	connect(tbConnectTo, SIGNAL(returnPressed()), this, SLOT(btnConnect_clicked()));
	//connect(btnCancel, SIGNAL(clicked()), this, SLOT(btnCancel_clicked()));
	connect(btnConnect, SIGNAL(clicked()), this, SLOT(btnConnect_clicked()));
	return moduleWidget;
}

// 	SESSION_TRY_START(session);
// 	session->lock(PROPERTY_YELLOWPAGES, HERE);
// 	RYellowPages* yp = session->getObjectAsL<RYellowPages>(PROPERTY_YELLOWPAGES);
// 	if (yp) {
// 		const map<Network::NetProtocol, map<string, Network::SocketAddress> >& ypmap = yp->getMap();
// 		map<Network::NetProtocol, map<string, Network::SocketAddress> >::const_iterator tcpStuff
// 			= ypmap.find(Network::TCP_IP);
// 		if (tcpStuff != ypmap.end()) {
// 			for (map<string, Network::SocketAddress>::const_iterator it = tcpStuff->second.begin();
// 			it != tcpStuff->second.end(); ++it) {
// 				try {
// 					Network::InetAddress inetaddr = Network::InetAddress(it->second);
// 					new QListBoxText(lbYpHosts, QString("%1 (%2:%3)")
// 						.arg(it->first)
// 						.arg(inetaddr.getIPAddress())
// 						.arg(inetaddr.getPort()));
// 				}
// 				catch (const Network::NetException&e) {
// 					RDK_ERROR_PRINTF(e.fullwhat());
// 				}
// 			}
// 		}
// 	}
// 	session->unlock(PROPERTY_YELLOWPAGES);
// 	SESSION_END_CATCH_TERMINATE(session);
// 
// 	connect(tbConnectTo, SIGNAL(returnPressed()), this, SLOT(btnConnect_clicked()));
// 	connect(btnCancel, SIGNAL(clicked()), this, SLOT(btnCancel_clicked()));
// 	connect(btnConnect, SIGNAL(clicked()), this, SLOT(btnConnect_clicked()));
// 	connect(lbYpHosts, SIGNAL(clicked(QListBoxItem*)), this, SLOT(lbYpHosts_clicked(QListBoxItem*)));
// 
// 	SESSION_TRY_START(session)
// 		Common::registerModuleWidget(session, getModuleName(), widget);
// 	SESSION_END_CATCH_TERMINATE(session)
// 
// 	return true;
// }

void RemoteConnectorModule::btnConnect_clicked()
{
	string connectTo = tbConnectTo->text();
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
	guiSession->tryConnectTo(Network::TCP_IP, connectTo);
	SESSION_END_CATCH_TERMINATE(guiSession)
	RqCommon::createTreeWidget(connectTo);
/*	string connectTo = tbConnectTo->text();
	SESSION_TRY_START(qtSession)
		qtSession->lock(PROPERTY_YELLOWPAGES, HERE);
		RYellowPages* yp = qtSession->getObjectAsL<RYellowPages>(PROPERTY_YELLOWPAGES);
		if (yp) {
			Network::SocketAddress sadd;
			bool found = yp->getSocketAddress(Network::TCP_IP, connectTo, sadd);
			if (found) {
				RDK_DEBUG_PRINTF("Connecting to '%s' (found on yellow pages)", connectTo.c_str());
				qtSession->tryConnectTo(Network::TCP_IP, connectTo);
				Common::showTree(qtSession, connectTo);
			}
			else {
				QRegExp rex = QRegExp("([^:]*):(\\d*)");
					rex.search(connectTo);
				RDK_DEBUG_PRINTF("Cannot find '%s' on yellow pages, adding a new yp item (ip: %s, port %d)",
					connectTo.c_str(), rex.cap(1).latin1(), rex.cap(2).toInt());
			}
		}
		qtSession->unlock(PROPERTY_YELLOWPAGES);
	SESSION_END_CATCH_TERMINATE(qtSession)*/
}

void RemoteConnectorModule::lbYpHosts_clicked(QListBoxItem*)
{
	QRegExp rex = QRegExp("(\\w*) \\(([^:]*):(\\d*)\\)");
	rex.search(lbYpHosts->currentText());
	tbConnectTo->setText(rex.cap(1));
}

void RemoteConnectorModule::btnCancel_clicked()
{
	RDK_DEBUG_PRINTF("Suicide");
}

void RemoteConnectorModule::exec()
{
	while (session->wait(), !exiting) {
		// we will never be here, since we aren't listening to anything
	}
}

MODULE_FACTORY(RemoteConnectorModule);

}} // ns
