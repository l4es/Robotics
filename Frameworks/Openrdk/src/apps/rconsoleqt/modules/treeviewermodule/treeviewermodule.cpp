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

#include <qlabel.h>
#include <qapplication.h>

#include <rdkcore/serialization_xml/xmlwriter.h>
#include <rdkcore/repository_struct/rpropertydef.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "TreeViewerModule"

#include "treeviewermodule.h"

#include "../../rqcommon/rqcommon.h"

#define DEBUG_THIS
#ifdef DEBUG_THIS
#define DEBUG_PRINTF(a, b...) RDK_DEBUG_PRINTF(a, ##b)
#else
#define DEBUG_PRINTF(a, b...)
#endif

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::Serialization::Xml;

bool TreeViewerModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		session->createString("agentName", "");
	SESSION_END_CATCH_TERMINATE(session)
	return true;
}

bool TreeViewerModule::init()
{
	QApplication::postEvent(this, new QCustomEvent(RqCommon::EVENT_CREATE_WIDGET));

	SESSION_TRY_START(session)
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
	return true;
}

void TreeViewerModule::cleanup()
{
	QWidget* widget = RqCommon::getModuleWidget(getModuleName());
	RqCommon::unregisterModuleWidget(getModuleName());
	widget->deleteLater();		// XXX I'm not sure if this is safe to be called outside the QT thread
}

void TreeViewerModule::customEvent(QCustomEvent* e)
{
	if (e->type() == RqCommon::EVENT_CREATE_WIDGET) {
		RqCommon::registerModuleWidget(getModuleName(), createWidget());
		widgetReady = true;
	}
}

QWidget* TreeViewerModule::createWidget()
{
	QT_THREAD_GUARD(0)
	
	QWidget* moduleWidget = new QWidget();
	QHBoxLayout* layout = new QHBoxLayout(moduleWidget);
	
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
			treeViewerWidget = new TreeViewerWidget(moduleWidget, guiSession->getString("/" + getModuleName() + "/agentName"));
		//printf("%s\n", guiSession->getString("/" + getModuleName() + "/agentName").c_str());
	SESSION_END_CATCH_TERMINATE(guiSession)
	
	layout->addWidget(treeViewerWidget);

	moduleWidget->setMinimumSize(400, 400);
	return moduleWidget;
}

void TreeViewerModule::exec()
{
	SESSION_TRY_START(session)
		// we ask the property tree
		session->queuePush(PROPERTY_OUTBOX, new RNetMessage(session->getString("agentName"),
			session->getRepositoryName(), RNetMessage::AGENT_CMD, Network::TCP_IP,
			new RString("Repository::getPropertyTree")));
		DEBUG_PRINTF("Asked the tree to agent '%s'", session->getString("agentName").c_str());
	SESSION_END_CATCH_TERMINATE(session)

	WIDGET_GUARD
			
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)
		vector<const RNetMessage*> v = session->queueFreezeAs<RNetMessage>(PROPERTY_INBOX);
		for (size_t i = 0; i < v.size(); i++) {
			DEBUG_PRINTF("Got a property tree message for me (%s)", session->getString("agentName").c_str());
			const RDK2::Object* obj = v[i]->getPayload();
			const RPropertyDefVector* defs = dynamic_cast<const RPropertyDefVector*>(obj);
			if (defs) {
				treeViewerWidget->buildTree(defs);
				Repository* repos = session->getRepository();
				for (RPropertyDefVector::const_iterator it = defs->begin(); it != defs->end(); ++it) {
					repos->setRemotePropertyDef((*it)->completeUrl, *it);
				}
			}
			else {
				RDK_ERROR_PRINTF("There is a weird error in dynamic casting a RPropertyDefVector");
			}
		}

		vector<const Event*> events = session->events();
		for (size_t i = 0; i < events.size(); i++) {
			if (events[i]->type == "EventPropertyTreeAdded") {
				const EventPropertyTreeAdded* e = 
					dynamic_cast<const EventPropertyTreeAdded*>(events[i]);
				if (e) {
					string host = e->propertyUrl.getHost();
					if (host == session->getString("agentName")
					|| (host == "" && session->getString("agentName") ==
					session->getRepositoryName())) {
						Url absoluteUrl = e->propertyUrl.getPath();
						string desc = session->getPropertyDescription(absoluteUrl);
						string linkto = session->getPropertyLinkTo(absoluteUrl);
						string classname = session->getPropertyClassName(absoluteUrl);
						treeViewerWidget->addProperty(absoluteUrl, classname, desc, linkto);
					}
				}
			}
			else if (events[i]->type == "EventPropertyTreeDeleted") {
				const EventPropertyTreeDeleted* e = 
					dynamic_cast<const EventPropertyTreeDeleted*>(events[i]);
				if (e) {
					treeViewerWidget->deleteProperty(e->propertyUrl);
				}
			}
		}
		treeViewerWidget->postRefresh();
		SESSION_END_CATCH_TERMINATE(session)
	}
}

MODULE_FACTORY(TreeViewerModule);

}} // ns
