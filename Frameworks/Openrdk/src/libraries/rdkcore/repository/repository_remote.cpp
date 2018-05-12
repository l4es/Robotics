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

#include <rdkcore/rnetobjects/rnetmessage.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "Repository"

#include "repository.h"
#include <rdkcore/rgraphics/rimagediffrect.h>
#include <rdkcore/rmaps/rmapimagediffrect.h>

#include <rdkcore/repository_struct/rpropertydef.h>
#include <rdkcore/repository_struct/rremotesubscription.h>
#include <rdkcore/repository_struct/rpropertyupdate.h>

//#define DOUT RDK_DEBUG_PRINTF
#define DOUT(args...)

namespace RDK2 { namespace RepositoryNS {

using RDK2::RNetObjects::RNetMessage;
using namespace RDK2::RGraphics;
using namespace RDK2::RMaps;

Property* Repository::getRemoteProperty(CUrl url, Session* askingSession) throw ()
{
	/*DOUT("Remotely getting property '%s' (asked by session %x, '%s')", url.c_str(), askingSession,
		askingSession ? askingSession->getSessionName().c_str() : "?");*/
	propertiesMutex.lock(HERE);
	map<Url, Property*>::iterator it = remoteProperties.find(url);
	if (it == remoteProperties.end()) {
		// it is the first time someone reference this remote property, there is no cache for it
		RDK_DEBUG_PRINTF("Creating cache for remote property '%s'", url.c_str());
		if (url.getProtocol() == "rdk") {
			DOUT("It's the first time someone refers to '%s': blind subscribing", url.c_str());
			Property* newProp = new Property(url, "");
			remoteProperties.insert(make_pair(url, newProp));
			RemoteSubscriptionRegister& reg = remoteSubscriptionRegister;
			RRemoteSubscription* sub = new RRemoteSubscription;
			sub->completeUrl = url;
			sub->askingHost = getRepositoryName();
			sub->socketType = Network::UDP_IP;
			reg.addPropertyToReceiveSubscription(askingSession, sub);
			propertiesMutex.unlock();
			return newProp;
		}
		else {
			ApplicationProtocolHandler* aph = getApplicationProtocolHandler(url.getProtocol());
			if (!aph) {
				RDK_WARNING_PRINTF("No registered handler for application protocol '%s' (property '%s')",
					url.getProtocol().c_str(), url.c_str());
			}
			Property* newProperty = new Property(url, "");
			remoteProperties.insert(make_pair(url, newProperty)); 
			propertiesMutex.unlock();
			if (aph) aph->remotePropertyRequested(askingSession, url);
			return newProperty;
		}
	}
	else {
		propertiesMutex.unlock();
		if (url.getProtocol() == "rdk") {
			// XXX forse questo �troppo lento da fare a ogni richiesta
			// magari �molto pi veloce mettere un bool nella Property, o un vector, o una list, o...
			RemoteSubscriptionRegister& reg = remoteSubscriptionRegister;
			vector<RRemoteSubscription> subs =
				reg.queryForPropertiesToReceive(url, "", Network::NETPROTOCOL_ANY,
				RRemoteSubscription::WHAT_ANY, RRemoteSubscription::WHEN_ANY);
			if (subs.empty()) {
				DOUT("It's the first time someone wants to see '%s': blind subscribing", url.c_str());
				RemoteSubscriptionRegister& reg = remoteSubscriptionRegister;
				RRemoteSubscription* sub = new RRemoteSubscription;
				sub->completeUrl = url;
				sub->askingHost = getRepositoryName();
				sub->socketType = Network::UDP_IP;
				reg.addPropertyToReceiveSubscription(askingSession, sub);
			}
			else {
				subs = reg.queryForPropertiesToReceive(url, "", Network::TCP_IP,
					RRemoteSubscription::WHAT_ANY, RRemoteSubscription::WHEN_ANY);
				if (subs.size() > 0) {
					DOUT("Someone requested a TCP remote subscription for property '%s'",
						url.c_str());
					if (tcpManager
					&& tcpManager->getConnectionStatusWith(url.getHost()) != Network::CONNECTED) {
						DOUT("...and there is no TCP connection with '%s', trying",
							url.getHost().c_str());
						tcpManager->tryConnectTo(url.getHost());
					}
				}
			}
		}
		else {
			ApplicationProtocolHandler* aph = getApplicationProtocolHandler(url.getProtocol());
			if (!aph) {
				RDK_WARNING_PRINTF("No registered handler for application protocol '%s' (property '%s')",
					url.getProtocol().c_str(), url.c_str());
			}
			if (aph) aph->remotePropertyRequested(askingSession, url);
		}
		return it->second;
	}
}

void Repository::sendObjectsToRemoteQueueSubscribers(CUrl completeUrl, const RDK2::Object* objToSend) throw()
{
	RemoteSubscriptionRegister* reg = &remoteSubscriptionRegister;
	vector<RRemoteSubscription> psendSub = reg->queryForPropertiesToSend(completeUrl, "",
		Network::NETPROTOCOL_ANY, RRemoteSubscription::QUEUE, RRemoteSubscription::WHEN_ANY);
	for (vector<RRemoteSubscription>::iterator it = psendSub.begin(); it != psendSub.end(); ++it) {
		RDK_DEBUG_PRINTF("Sending message %s to %s",completeUrl.c_str(),it->askingHost.c_str());
		queuePush(PROPERTY_OUTBOX,
			new RNetMessage(it->askingHost, getRepositoryName(),
			RNetMessage::PROPERTY_UPDATE, it->socketType,
			new RPropertyUpdate(completeUrl, RPropertyUpdate::ENQUEUE,
			objToSend->clone())));
	}
}

void Repository::sendStorageValuesToRemoteValueSubscribers(CUrl completeUrl, cstr askingHost,
	Network::NetProtocol socketType, RRemoteSubscription::What what,
	RRemoteSubscription::When when) throw ()
{
	RemoteSubscriptionRegister* reg = &remoteSubscriptionRegister;
	vector<RRemoteSubscription> psendSub =
		reg->queryForPropertiesToSend(completeUrl, askingHost, socketType, what, when);
	for (vector<RRemoteSubscription>::iterator it = psendSub.begin(); it != psendSub.end(); ++it) {
		Timestamp now;
		if ((now - it->lastSendingTimestamp).getMs() >= it->maxPeriod) {
			sendStorageValueToHost(it->completeUrl, it->askingHost, it->socketType);
			reg->refreshLastSendingTimestamp(it->id);
		}
	}
}

void Repository::sendStorageValueToHost(CUrl completeUrl, cstr askingHost,
	Network::NetProtocol socketType, RDK2::Object* objToSend)
	throw ()
{
	//RDK_DEBUG_PRINTF("Want to send %s to %s, using protocol %d", completeUrl.c_str(), askingHost.c_str(), socketType);
	bool alreadyunlocked = false;	// FIXME AAAAAAHHHHHHHH
	try {
		lock(completeUrl, WHERE("Sending '" + completeUrl + "' value to '" + askingHost + "'"));
		try {
			if (socketType == Network::TCP_IP) {
				DOUT("Sending property '%s' via TCP", completeUrl.c_str());
				if (!objToSend){ 
					const RDK2::Object* objts = getObjectLC(completeUrl);
					if (objts) {
#if !defined OpenRDK_ARCH_GEODE && !defined OpenRDK_ARCH_ATOM
						if (objts->getClassName() == "RImage") {
							const RImage* rimg = dynamic_cast<const RImage*>(objts);
							if (rimg) {
								objToSend = rimg->convertTo(RImage::JPEG);
							}
							else {
								RDK_ERROR_PRINTF("Cannot dynamic cast the RImage");
								objToSend = 0;
							}
						}
						else
#endif
						{
							objToSend = objts->clone();
						}
					}
					else objToSend = 0;
				}
				if (objToSend) {
					/*RDK_DEBUG_PRINTF("Sending storage value (TCP) for property '%s' to '%s'",
						completeUrl.c_str(), askingHost.c_str());*/
					queuePush(PROPERTY_OUTBOX,
						new RNetMessage(askingHost, getRepositoryName(),
						RNetMessage::PROPERTY_UPDATE, socketType,
						new RPropertyUpdate(completeUrl, RPropertyUpdate::VALUE,
						objToSend)));
				}
			}
			else if (socketType == Network::UDP_IP) {
				Property* p = getProperty(completeUrl);
				string objClass = p->getObjectClassName();
				bool isVector = (objClass.find("Vector") != string::npos);
				if (p->getObjectClassName() != "RImage" && p->getObjectClassName() != "RMapImage"
				&& !isVector) {
					if (!objToSend) {
						const RDK2::Object* sourceObj = getObjectLC(completeUrl);
						if (sourceObj) objToSend = sourceObj->clone();
					}
					if (objToSend) {
						/*RDK_DEBUG_PRINTF("Sending storage value (UDP) for property '%s' to '%s'",
							completeUrl.c_str(), askingHost.c_str());*/
						queuePush(PROPERTY_OUTBOX,
							new RNetMessage(askingHost, getRepositoryName(),
							RNetMessage::PROPERTY_UPDATE, socketType,
							new RPropertyUpdate(completeUrl, RPropertyUpdate::VALUE,
							objToSend)));
						// XXX ? (avoids the burst and thus the udp buffer overflow)
						//usleep(1);
					}
				}
				else if (p->getObjectClassName() == "RImage") {
					objToSend = getObjectL(completeUrl);
					if (objToSend) {
						int maxContentSize = 1000; // FIXME millemila
						vector<ObjectDiff*> diffs = objToSend->splitInDiffs(maxContentSize);
						alreadyunlocked = true;
						unlock(completeUrl);
						/*RDK_DEBUG_PRINTF("Sending storage value (UDP) for property '%s' "
							"to '%s' split in %u diffs", completeUrl.c_str(), askingHost.c_str(),
							diffs.size());*/
						//int counter = 0;
						int offset = rand() % diffs.size();
						for (size_t i = 0; i < diffs.size(); i++) {
							RImageDiffRect* imgdiffr = dynamic_cast<RImageDiffRect*>(diffs[i]);
							if (imgdiffr) {
								//RDK_DEBUG_PRINTF("buffer size: %u", imgdiffr->getCBufSize());
							}
							// FIXME fare chiarezza
							int a = (i+offset)%diffs.size();
							
							// AC: a % n  e' compreso tra 0 e n-1 se a e' positivo.
							// if (a > diffs.size()-1) a = diffs.size()-1;
							// else if (a < 0) a = 0;
							//RDK_DEBUG_PRINTF("Il tuo dio %d", a);
							a = i;
							
							queuePush(PROPERTY_OUTBOX,
								new RNetMessage(askingHost, getRepositoryName(),
								RNetMessage::PROPERTY_UPDATE, socketType,
								new RPropertyUpdate(completeUrl, RPropertyUpdate::DIFF,
								diffs[a])));
								// XXX ? (avoids burst and udp buffer overflow)
								//if ((counter++)%20==0) usleep(1);
						}
					}
				}
				else if (p->getObjectClassName() == "RMapImage") {
					objToSend = getObjectL(completeUrl);
					if (objToSend) {
						int maxContentSize = 1000; // FIXME millemila
						vector<ObjectDiff*> diffs = objToSend->splitInDiffs(maxContentSize);
						alreadyunlocked = true;
						unlock(completeUrl);
						/*RDK_DEBUG_PRINTF("Sending storage value (UDP) for property '%s' "
							"to '%s' split in %u diffs", completeUrl.c_str(), askingHost.c_str(),
							diffs.size());*/
						//int counter = 0;
						for (size_t i = 0; i < diffs.size(); i++) {
							RMapImageDiffRect* imgdiffr = dynamic_cast<RMapImageDiffRect*>(diffs[i]);
							if (imgdiffr) {
								/*RDK_DEBUG_PRINTF("Diff %d; buffer size: %u",
									i, imgdiffr->imageDiffRect->getCBufSize());*/
							}
							queuePush(PROPERTY_OUTBOX,
								new RNetMessage(askingHost, getRepositoryName(),
								RNetMessage::PROPERTY_UPDATE, socketType,
								new RPropertyUpdate(completeUrl, RPropertyUpdate::DIFF,
								diffs[i])));
								// XXX ? (avoids burst and udp buffer overflow)
								//if ((counter++)%20==0) usleep(1);
						}
					}
				}
				else if (isVector) {
					objToSend = getObjectL(completeUrl);
					if (objToSend) {
						int maxContentSize = 1000; // FIXME millemila
						vector<ObjectDiff*> diffs = objToSend->splitInDiffs(maxContentSize);
						alreadyunlocked = true;
						unlock(completeUrl);
						/*RDK_DEBUG_PRINTF("Sending storage value (UDP) for property '%s' "
							"to '%s' split in %u diffs", completeUrl.c_str(), askingHost.c_str(),
							diffs.size());*/
						//int counter = 0;
						for (size_t i = 0; i < diffs.size(); i++) {
							queuePush(PROPERTY_OUTBOX,
								new RNetMessage(askingHost, getRepositoryName(),
								RNetMessage::PROPERTY_UPDATE, socketType,
								new RPropertyUpdate(completeUrl, RPropertyUpdate::DIFF,
								diffs[i])));
								// XXX ? (avoids burst and udp buffer overflow)
								//if ((counter++)%20==0) usleep(1);
						}
					}
				}
			}
			else {
				RDK_ERROR_PRINTF("You should decide how to send this stuff!");
			}
		}
		catch (const SessionException& e) {
			RDK_ERROR_PRINTF(e.what());
		}
		if (!alreadyunlocked) unlock(completeUrl);
	}
	catch (const SessionException& e) {
		RDK_ERROR_PRINTF(e.what());
	}
}

void Repository::sendStorageDiffToHost(CUrl completeUrl, cstr askingHost, Network::NetProtocol socketType,
	ObjectDiff* diffToSend) throw ()
{
	if (!diffToSend) {
		RDK_ERROR_PRINTF("Trying to send a null diff");
		return;
	}
	try {
		lock(completeUrl, HERE);
		try {
			if (socketType == Network::TCP_IP) {
				DOUT("Sending storage diff (TCP) for property '%s' to '%s'",
					completeUrl.c_str(), askingHost.c_str());
				queuePush(PROPERTY_OUTBOX,
					new RNetMessage(askingHost, getRepositoryName(),
					RNetMessage::PROPERTY_UPDATE, socketType,
					new RPropertyUpdate(completeUrl, RPropertyUpdate::DIFF,
					diffToSend)));
			}
			else if (socketType == Network::UDP_IP) {
				DOUT("Sending storage diff (UDP) for property '%s' to '%s'",
					completeUrl.c_str(), askingHost.c_str());
				int maxContentSize = 1000; // FIXME millemila
				vector<ObjectDiff*> diffs = diffToSend->split(maxContentSize);
				DOUT("Split diff in %u diffs", diffs.size());
				for (size_t i = 0; i < diffs.size(); i++) {
					RImageDiffRect* imgdiffr = dynamic_cast<RImageDiffRect*>(diffs[i]);
					if (imgdiffr) {
						//RDK_DEBUG_PRINTF("buffer size: %u", imgdiffr->getCBufSize());
					}
					queuePush(PROPERTY_OUTBOX,
						new RNetMessage(askingHost, getRepositoryName(),
						RNetMessage::PROPERTY_UPDATE, socketType,
						new RPropertyUpdate(completeUrl, RPropertyUpdate::DIFF,
						diffs[i])));
				}
				delete diffToSend;
			}
			else {
				RDK_ERROR_PRINTF("You should decide how to send this stuff!");
			}
		}
		catch (const SessionException& e) {
			RDK_ERROR_PRINTF(e.what());
		}
		unlock(completeUrl);
	}
	catch (const SessionException& e) {
		RDK_ERROR_PRINTF(e.what());
	}
}

void Repository::remotePropertyOptions(CUrl url, Network::NetProtocol socketType,
	RRemoteSubscription::What what, RRemoteSubscription::When when, double /*minPeriod*/, double maxPeriod)
	throw (NoSuchProperty, InvalidOperation)
{
	if (isLocalProperty(url)) return;

	propertiesMutex.lock(HERE);
	map<Url, Property*>::iterator it = remoteProperties.find(url);
	if (it == remoteProperties.end()) {
		remoteProperties.insert(make_pair(url, new Property(url, "")));
	}
	propertiesMutex.unlock();

	RemoteSubscriptionRegister& reg = remoteSubscriptionRegister;
	RRemoteSubscription* sub = new RRemoteSubscription;
	sub->completeUrl = url;
	sub->askingHost = getRepositoryName();
	sub->what = what;
	sub->when = when;
	sub->socketType = socketType;
	sub->minPeriod = maxPeriod;
	sub->maxPeriod = maxPeriod;
	reg.addPropertyToReceiveSubscription(0, sub);
}

void Repository::setRemotePropertyDef(CUrl url, const RPropertyDef* def)
{
	if (isLocalProperty(url)) return;

	propertiesMutex.lock(HERE);
	map<Url, Property*>::iterator it = remoteProperties.find(url);
	if (it == remoteProperties.end()) {
		/*RDK_DEBUG_PRINTF("Adding remote def property for '%s' (class %s)",
			url.c_str(), def->getObjectClassName().c_str());*/
		remoteProperties.insert(make_pair(url, new Property(*def)));
	}
	else {
		/*RDK_DEBUG_PRINTF("Setting remote daf property for '%s' (class %s)",
			url.c_str(), def->getObjectClassName().c_str());*/
		Property* p = it->second;
		p->lock(HERE);
		if (p->getObjectClassName() == "") p->setObjectClassName(def->getObjectClassName());
		// FIXME anche gli altri membri di property
		p->unlock();
	}
	// FIXME fare un posto comune per questo (vedi repository.cpp)
	for (multimap<string, Session*>::iterator it = sessionsListeningToTree.begin();
	it != sessionsListeningToTree.end(); ++it) {
		if (url.startsWith(it->first)) {
			it->second->pushEvent(new EventPropertyTreeAdded(url));
		}
	}
	propertiesMutex.unlock();
}

// propriet�locali storage
//   quando viene aggiornata
//     per ogni host che ha sottoscritto la propriet�FULL + WHEN_CHANGES, invia oggetto
//     per ogni host che ha sottoscritto la propriet�UPDATES + SINGLE, invia aggiornamento
//   quando viene letta
//     se non �ancora stata sottoscritta, sottoscriverla con opzioni di default

// propriet�locali queue
//   quando viene aggiornata
//     per ogni host che ha sottoscritto la propriet�HISTORY, invia gli oggetti non ancora inviati
//     per ogni host che ha sottoscritto la propriet�LAST, invia l'ultimo oggetto
//   quando viene letta
//     se non �stata ancora sottoscritta, sottoscriverla con opzioni di default

// propriet�remote
//   quando viene aggiornata
//     inviare l'aggiornamento all'host remoto
//   quando viene letta
//     se non �ancora stata sottoscritta, sottoscriverla con opzioni di default
//     prendere la copia locale

// thread periodico
//   per ogni propriet�che �stata sottoscritta FULL + PERIODIC, invia l'oggetto (secondo period)
//   per ogni propriet�che �stata sottoscritta UPDATES + PACK, invia gli aggiornamenti (comprimendoli e secondo maxWaitPeriod)

// ("inviare" = mettere nella outbox)

void Repository::registerApplicationProtocolHandler(const string& protocol, ApplicationProtocolHandler* handler)
{
	string prot = protocol.substr(0, protocol.find_first_of(":/"));
	applicationProtocolHandlersMutex.lock(HERE);

	map<string, ApplicationProtocolHandler*>::iterator it = applicationProtocolHandlers.find(prot);
	if (it != applicationProtocolHandlers.end()) {
		RDK_ERROR_PRINTF("Trying to register another application protocol handler for protocol '%s', only the first is valid",
			prot.c_str());
	}
	else {
		applicationProtocolHandlers.insert(make_pair(prot, handler));
	}

	applicationProtocolHandlersMutex.unlock();
}

ApplicationProtocolHandler* Repository::getApplicationProtocolHandler(const string& protocol)
{
	applicationProtocolHandlersMutex.lock(HERE);
	map<string, ApplicationProtocolHandler*>::iterator it = applicationProtocolHandlers.find(protocol);
	applicationProtocolHandlersMutex.unlock();
	if (it == applicationProtocolHandlers.end()) return NULL;
	else return it->second;
}

}} // namespaces
