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

#include <errno.h>

#include <float.h>

#include <rdkcore/object/objectmanager.h>
#include <rdkcore/rnetobjects/rnetmessage.h>
#include <rdkcore/rnetobjects/ryellowpages.h>
#include <rdkcore/repository_struct/rpropertyupdate.h>
#include <rdkcore/repository_struct/rremotesubscription.h>
#include <rdkcore/object/objectdiff.h>
#include <rdkcore/common/threads.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RemotePropertyManager"

#include "repository.h"

//#define DEBUG_THIS
#ifdef DEBUG_THIS
#warning a lot of debug
#define DOUT(a, args...) RDK_DEBUG_PRINTF(a, ## args)
#else
#define DOUT(a, args...)
#endif

namespace RDK2 { namespace RepositoryNS {

using namespace Network;
using namespace RDK2::Common;
using RDK2::RNetObjects::RNetMessage;
using RDK2::RNetObjects::RYellowPages;

RemotePropertyManager::RemotePropertyManager(Repository* repository) : repository(repository), exiting(false)
{ }

bool RemotePropertyManager::init()
{
	return true;
}

bool RemotePropertyManager::start()
{
	if (!createThread(&receiverThreadId, (void*(*)(void*))receiverThreadFn, this,
		"property updates receiver thread")) return false;
	if (!createThread(&periodicSenderThreadId, (void*(*)(void*))periodicSenderThreadFn, this,
		"property updates periodic sender thread")) return false;
	if (!createThread(&periodicUdpRenewalThreadId, (void*(*)(void*))periodicUdpRenewalThreadFn, this,
		"property updates periodic UDP subscriptions renewal thread")) return false;
	return true;
}

void RemotePropertyManager::receiver()
{
	RDK_INFO_PRINTF("Property updates receiver started");

	receiverSession = repository->createSession("propertyReceiver",
		"Property updates receiver session", "/");
	Session* session = receiverSession;	// just a shortcut

	SESSION_TRY_START(session)
	session->queueSubscribe(PROPERTY_INBOX);
	session->queueSetInterests(PROPERTY_INBOX, new RemotePropertyManagerInterests);
	session->listen(PROPERTY_INBOX);
	SESSION_END_CATCH_TERMINATE(session)

	while (session->wait(), !exiting) {
		try {
			session->start();
			vector<const RDK2::Object*> objects = session->queueFreeze(PROPERTY_INBOX);
			DOUT("Inbox contains %d messages", objects.size());
			for (vector<const RDK2::Object*>::iterator it = objects.begin();
			it != objects.end(); ++it) {
				try {
					const RNetMessage* msg = dynamic_cast<const RNetMessage*>(*it);
					if (!msg) {
						RDK_ERROR_PRINTF("Found something weird in the inbox (class: '%s')",
							(*it)->getClassName().c_str());
					}
					else {
						if (msg->getType() == RNetMessage::PROPERTY_UPDATE) {
							manageUpdateMessage(msg);
						}
						else if (msg->getType() == RNetMessage::PROPERTY_SUBSCRIPTION) {
							manageSubscriptionMessage(msg);
						}
					}
				}
				catch (const SessionException& e) {
					//RDK_ERROR_PRINTF(e.what());
				}
			}
			session->end();
		}
		catch (SessionException& exc) {
			session->terminate();
			RDK_ERROR_STREAM("Failure while taking inbox: " << exc.what());
			sleep(1);
		}
	}

	SESSION_TRY_START(session)
	session->queueUnsubscribe(PROPERTY_INBOX);
	SESSION_END_CATCH_TERMINATE(session)
}

void RemotePropertyManager::manageUpdateMessage(const RNetMessage* msg) throw (SessionException)
{
	const RPropertyUpdate* pu = dynamic_cast<const RPropertyUpdate*>(msg->getPayload());
	if (!pu) {
		RDK_ERROR_PRINTF("There wasn't a property update in the message "
			"(class: '%s')", msg->getPayload()->getClassName().c_str());
	}
	else {
		if (!pu->completeUrl.isComplete()) {
			RDK_ERROR_PRINTF("Url of property is not complete (%s)", pu->completeUrl.c_str());
		}
		else {
			DOUT("Managing update message for property '%s'", pu->completeUrl.c_str());
			switch (pu->type) {
				case RPropertyUpdate::VALUE: {
					repository->lock(pu->completeUrl, HERE);
					Property* p = repository->getProperty(pu->completeUrl, 0);
					if (p) {
						const RImage* rimg = dynamic_cast<const RImage*>(pu->object);
						if (rimg) {
							if (rimg->getType() == RImage::JPEG) {
								RImage* r24 = rimg->convertTo(RImage::RGB24);
								if (r24) p->setObject(r24);
							}
							else p->setObject((RImage*) rimg->clone());
						}
						else p->setObject(pu->object->clone());
					}
					repository->unlock(pu->completeUrl);
					repository->wakeUpListeningSessions(pu->completeUrl, "EventPropertyUpdateValue");
					// NOTE: no remote stuff (do not call repository->valueChanged())
				} break;
				case RPropertyUpdate::DIFF: {
					repository->lock(pu->completeUrl, HERE);
					Property* p = repository->getRemoteProperty(pu->completeUrl, 0);
					const ObjectDiff* diff = dynamic_cast<const ObjectDiff*>(pu->object);
					if (diff) {
						//RDK_DEBUG_PRINTF("Diff class: %s", diff->objectClassName.c_str());
						if (p->getObjectClassName() == "") p->setObjectClassName(diff->objectClassName);
						RDK2::Object* obj = repository->getObjectOrCreateItL(pu->completeUrl);
						obj->applyDiff(diff);
						//RDK_DEBUG_PRINTF("Diff applied");
					}
					else {
						RDK_ERROR_PRINTF("It wasn't an ObjectDiff (but a '%s')",
							pu->object->getClassName().c_str());
					}
					repository->unlock(pu->completeUrl);
					repository->pushDiffInDiffQueue(pu->completeUrl, (ObjectDiff*) diff->clone());
					repository->wakeUpListeningSessions(pu->completeUrl, "EventPropertyUpdateDiff");
					// no remote stuff (do not call repository->valueChanged())
				} break;
				case RPropertyUpdate::ENQUEUE: {
					repository->queuePush(pu->completeUrl, pu->object->clone());
				} break;
			}
		}
	}
}

void RemotePropertyManager::manageSubscriptionMessage(const RNetMessage* msg) throw (SessionException)
{
	const RRemoteSubscription* remsub = dynamic_cast<const RRemoteSubscription*>(msg->getPayload());
	if (!remsub) {
		RDK_ERROR_PRINTF("There wasn't a remote subscription in the message "
			"(class: '%s')", msg->getPayload()->getClassName().c_str());
	}
	else {
		RRemoteSubscription* sub = (RRemoteSubscription*) remsub->clone();
		if (msg->getSender() == remsub->completeUrl.getHost()) {
			// the sender is sending a subscription for his own property:
			// this is an answer to an incomplete subscription
			DOUT("Managing answer to an incomplete subscription message for property '%s'",
				remsub->completeUrl.c_str());
			repository->remoteSubscriptionRegister.completePropertyToReceiveSubscription(sub);
		}
		else {
			DOUT("Managing remote subscription message for property '%s'", remsub->completeUrl.c_str());

			// send immediately the current object if VALUE+ON_CHANGE
			// or the object on which the diff have to be applied if UPDATES+ON_CHANGE
			bool reallyAdded = repository->remoteSubscriptionRegister.addPropertyToSendSubscription(sub);
			if (reallyAdded) {
				if (sub->when == RRemoteSubscription::ON_CHANGE) {
					repository->sendStorageValueToHost(sub->completeUrl, sub->askingHost, sub->socketType);
				}
			}
		}
	}
}

// greatest commond divisor (FIXME move this elsewhere)
int gcd(int n, int m)
{
	if (n == m) return n;
	else {
		int nave, k;
		while (n != m) {
			if (n > m) {
				nave = m;
				m = n;
				n = nave;
			}
			k = m - n;
			m = n;
			n = k;
			if (n == m) {
				k = n;
				return k;
			}
		}
	}
	return 0;	// FIXME?
}

void RemotePropertyManager::periodicSender()
{
	RDK_INFO_PRINTF("Property updates periodic sender started");

	periodicSenderSession = repository->createSession("propertyPeriodicSender",
		"Property updates periodic sender session", "/");
	Session* session = periodicSenderSession;	// just a shortcut

	while (!exiting) {
		SESSION_TRY_START(session)
			RemoteSubscriptionRegister* reg = &(repository->remoteSubscriptionRegister);

			// delete old udp subscription not renewed
			double timeoutMs = session->getDouble(PROPERTY_UDP_RENEWAL_TIMEOUT);
			struct timeval tvnow;
			gettimeofday(&tvnow, 0);
			double nowMs = tvnow.tv_sec * 1000 + tvnow.tv_usec / 1000;
			vector<RRemoteSubscription> psendSub = reg->queryForPropertiesToSend("", "", Network::UDP_IP,
				RRemoteSubscription::WHAT_ANY, RRemoteSubscription::WHEN_ANY);
			for (vector<RRemoteSubscription>::iterator it = psendSub.begin(); it != psendSub.end(); ++it) {
				double timestampMs = it->timestamp.tv_sec * 1000 + it->timestamp.tv_usec / 1000;
				if (nowMs - timestampMs > timeoutMs) {
					DOUT("Subscription [%s] has reached timeout, removing",
						it->getStringForVisualization().c_str());
					reg->deleteSubscriptionToSend(it->id);
				}
			}

			// send periodic storage values
			repository->sendStorageValuesToRemoteValueSubscribers("", "", Network::NETPROTOCOL_ANY,
				RRemoteSubscription::STORAGE_VALUE, RRemoteSubscription::PERIODIC);

			// send packed storage updates
			psendSub = reg->queryForPropertiesToSend("", "", Network::NETPROTOCOL_ANY,
				RRemoteSubscription::STORAGE_DIFFS, RRemoteSubscription::PERIODIC);
			for (vector<RRemoteSubscription>::iterator it = psendSub.begin(); it != psendSub.end(); ++it) {
				DOUT("Sending storage updates packed for property '%s' to '%s'",
					it->completeUrl.c_str(), it->askingHost.c_str());
				// FIXME we should merge the diffs
				//session->queuePush(PROPERTY_OUTBOX, ??);	// FIXME who has the diffs?
			}
		SESSION_END_CATCH_TERMINATE(session)
		usleep(100 * 1000);	// FIXME
	}

	delete session;
}

void RemotePropertyManager::periodicUdpRenewal()
{
	RDK_INFO_PRINTF("Property upd subscriptions renewal started");

	periodicUdpRenewalSession = repository->createSession("propertyUdpRenewer",
		"Property updates UDP renewal session", "/");
	Session* session = periodicUdpRenewalSession;	// just a shortcut

	while (!exiting) {
		double interval = 1000.;	// XXX
		SESSION_TRY_START(session)
			RemoteSubscriptionRegister* reg = &(repository->remoteSubscriptionRegister);
			vector<RRemoteSubscription> subToRenew = reg->queryForPropertiesToReceive("", "", Network::UDP_IP,
				RRemoteSubscription::WHAT_ANY, RRemoteSubscription::WHEN_ANY);
			for (vector<RRemoteSubscription>::iterator it = subToRenew.begin(); it != subToRenew.end(); ++it) {
				DOUT("Remotely refreshing udp subscription for property '%s' to '%s'",
					it->completeUrl.c_str(), it->completeUrl.getHost().c_str());
				session->queuePush(PROPERTY_OUTBOX,
					new RNetMessage(it->completeUrl.getHost(), session->getRepositoryName(),
					RNetMessage::PROPERTY_SUBSCRIPTION, Network::UDP_IP, it->clone()));
			}
			interval = session->getDouble(PROPERTY_UDP_RENEWAL_INTERVAL);
		SESSION_END_CATCH_TERMINATE(session)
		usleep((int) interval * 1000);
	}

	delete session;
}

RemotePropertyManager::~RemotePropertyManager()
{
	exiting = true;
	receiverSession->wakeUp();
	periodicSenderSession->wakeUp();
	periodicUdpRenewalSession->wakeUp();
	pthread_join(receiverThreadId, 0);
	pthread_join(periodicSenderThreadId, 0);
	pthread_join(periodicUdpRenewalThreadId, 0);
	RDK_INFO_PRINTF("Remote property manager closed");
}

}} // namespaces
