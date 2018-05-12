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

#ifndef RDK2_RQM_TREE_VIEWER_MODULE
#define RDK2_RQM_TREE_VIEWER_MODULE

#include <qlineedit.h>
#include <qpushbutton.h>
#include <qlayout.h>

#include <rdkcore/modules/module.h>
#include <rdkcore/posixqueues/interests.h>

#include "treeviewerwidget.h"

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RepositoryNS;
using namespace RDK2::RAgent;

class TreeViewerModule : public QObject, public Module {
private:
	TreeViewerWidget* treeViewerWidget;

	class MyInterests : public PosixQueues::MyInterests<RDK2::Object> {
	private:
		string agentName;

	public:
		MyInterests(cstr agentName) : agentName(agentName) { }
		
		// note: in this way, we cannot discriminate among different requests coming from the same agent, for example
		// when multiple "local repository" trees are open in the same rconsole;
		// explain why you should have multiple instances of the same tree!
		bool areYouInterested(const RDK2::Object* obj) {
			const RNetMessage* msg = dynamic_cast<const RNetMessage*>(obj);
			if (!msg) return false;
			else if (msg->getType() != RNetMessage::PROPERTY_TREE) return false;
			else if (msg->getSender() != agentName) return false;
			else return true;
		}
	};

	QWidget* createWidget();
	volatile bool widgetReady;

	void customEvent(QCustomEvent* e);
	
public:
	TreeViewerModule() : widgetReady(false) { }
	bool initConfigurationProperties();
	bool init();
	void exec();
	void cleanup();
};

}} // ns

#endif
