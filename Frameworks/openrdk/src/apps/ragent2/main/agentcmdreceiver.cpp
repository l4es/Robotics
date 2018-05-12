/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
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

#include <rdkcore/rprimitive/rstring.h>
using namespace RDK2::RPrimitive;

#include <rdkcore/serialization_xml/xmlwriter.h>
#include <rdkcore/repository_struct/rpropertydefvector.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "AgentCmdReceiver"

#include "agentcmdreceiver.h"

namespace RDK2 { namespace RAgent {


AgentCmdReceiver::AgentCmdReceiver(Repository* repository, ModuleManager* moduleManager) :
	exiting(false), moduleManager(moduleManager), threadId(0), session(0)
{
	session = repository->createSession("agentCmdReceiver", "Agent command receiver session", "/");
}

AgentCmdReceiver::~AgentCmdReceiver()
{
	delete session;
}

bool AgentCmdReceiver::init()
{
	SESSION_TRY_START(session)
		session->queueSubscribe(PROPERTY_INBOX);
		session->listen(PROPERTY_INBOX);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void AgentCmdReceiver::exec()
{
	//RDK_DEBUG_PRINTF("Agent receiver starting");
	while (session->wait(), !exiting) {
		//RDK_DEBUG_PRINTF("Agent receiver awaken");
		SESSION_TRY_START(session)
		vector<const RNetMessage*> v = session->queueFreezeAs<RNetMessage>(PROPERTY_INBOX);
		for (size_t i = 0; i < v.size(); i++) {
			if (v[i]->getType() != RNetMessage::AGENT_CMD) continue;	// FIXME filter!
			const RString* s = v[i]->getPayloadAs<RString>();
			if (s) {
				if (s->value == "Repository::getPropertyTree") {
					//RDK_DEBUG_PRINTF("NO CAPITO");
					map<Url, string> props = session->getRepository()->getLocalPropertyNames();
					RPropertyDefVector* prv = new RPropertyDefVector();
					for (map<Url, string>::iterator it = props.begin(); it != props.end(); ++it) {
						prv->push_back(session->getRPropertyDef(it->first));
					}
					RDK_DEBUG_PRINTF("Sending the property tree");
					session->queuePush(PROPERTY_OUTBOX, new RNetMessage(v[i]->getSender(),
						session->getRepository()->getRepositoryName(), RNetMessage::PROPERTY_TREE,
						v[i]->getNetProtocol(), prv));
/*					RDK2::Serialization::Xml::XmlWriter xw(true);
					RDK_DEBUG_STREAM("Sending " << xw.serialize(true, prv));*/
				}
				else {
					//RDK_DEBUG_STREAM("Got agent command '"<<s->text<<"'");
					moduleManager->moduleListMutex.lock(HERE);
					try {
						for (list<Module*>::iterator it = moduleManager->modules.begin();
						it != moduleManager->modules.end(); ++it) {
							(*it)->asyncAgentCmd(s->value);
						}
					}
					catch (...) {
						RDK_ERROR_PRINTF("An async command let an exception pass through (?!?)");
					}
					moduleManager->moduleListMutex.unlock();
				}
			}
		}
		SESSION_END_CATCH_TERMINATE(session)
	}
}

void AgentCmdReceiver::start()
{
	pthread_create(&threadId, 0, (void*(*)(void*)) AgentCmdReceiver::threadExec, this);
}

void AgentCmdReceiver::end()
{
	SESSION_TRY_START(session)
	exiting = true;
	session->wakeUp();
	pthread_join(threadId, 0);
	SESSION_END_CATCH_TERMINATE(session)
}

void* AgentCmdReceiver::threadExec(AgentCmdReceiver* instance)
{
	instance->exec();
	return 0;
}

}} // namespace
