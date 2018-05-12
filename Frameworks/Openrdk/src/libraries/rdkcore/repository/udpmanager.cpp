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

#include <rdkcore/object/objectmanager.h>
#include <rdkcore/rnetobjects/rnetmessage.h>
#include <rdkcore/rnetobjects/ryellowpages.h>
#include <rdkcore/common/threads.h>
#include <rdkcore/serialization_binary/binaryreader.h>
#include <rdkcore/serialization_binary/binarywriter.h>

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "UdpManager"

#include "repository.h"
#include "udpmanager.h"

//#define UDPMANAGER_DEBUG_RECEIVER
//#define UDPMANAGER_DEBUG_SENDER

namespace RDK2 { namespace RepositoryNS {

using namespace Network;
using namespace RDK2::Common;
using RDK2::RNetObjects::RNetMessage;
using RDK2::RNetObjects::RYellowPages;
using namespace RDK2::Serialization::Binary;

UdpManager::UdpManager(Repository* repository) : repository(repository), exiting(false),
	udpSocket()
{ }

bool UdpManager::init()
{
	receiverSession = repository->createSession("udpReceiver", "UDP receiver session", "/");
	senderSession = repository->createSession("udpSender", "UDP sender session", "/");

	Session* session = repository->createSession("udpPortBinder",
		"Session for UDP port binding", "/");

	bool done = false;
	while (!done) {
		try {
			session->start();

			InetAddress addrToBind;

			session->lock(PROPERTY_YELLOWPAGES, WHERE("Getting my address"));
			RYellowPages* yp = session->getObjectAsL<RYellowPages>(PROPERTY_YELLOWPAGES);
			if (yp->getSocketAddress(Network::UDP_IP, session->getRepositoryName(), addrToBind)) {
				session->setIntB(PROPERTY_UDP_PORT, addrToBind.getPort());
			}
			else {
				RDK_ERROR_PRINTF("I cannot find my address in the yellow pages");
			}
			session->unlock(PROPERTY_YELLOWPAGES);
			udpSocket.bind(addrToBind);
			RDK_INFO_PRINTF("UDP socket bound to port %d", addrToBind.getPort());

			done = true;
			session->end();
		}
		catch (Network::NetException& netexc) {
			session->terminate();
			RDK_ERROR_STREAM("Unable to bind UDP socket (" << netexc.fullwhat() << ", retry in 4s)");
			sleep(4);
		}
		catch (const SessionException& e) {
			session->terminate();
			delete session;
			RDK_ERROR_PRINTF("Error in getting yellow pages (%s)", e.what());
			return false;
		}
	}
	delete session;
	return true;
}

bool UdpManager::start()
{
	if (!createThread(&receiverThreadId, (void*(*)(void*))receiverThreadFn,
			this, "UDP receiver thread")) return false;
	if (!createThread(&senderThreadId, (void*(*)(void*))senderThreadFn,
			this, "UDP sender thread")) return false;
	return true;
}

void UdpManager::receiver() {
#ifdef UDPMANAGER_DEBUG_RECEIVER
	#warning UDPMANAGER_DEBUG_RECEIVER enabled ( A lot of debug, not good for everyday job )
	#define LDOUT(a) { RDK_DEBUG_STREAM("receiver: "<<a) }
#else
	#define LDOUT(a)
#endif
	RDK_INFO_PRINTF("UDP receiver started");

	Session* session = receiverSession;		// just a shortcut

	BinaryReader br;

	while (!exiting) {
		string message_buffer;
		Network::InetAddress from;
		int nbytes = 0;
		LDOUT("Waiting for incoming messages");
		LDOUT("Socket is bound to " << udpSocket.getAddress().getIPAddress().c_str() << ", port "
			<< udpSocket.getAddress().getPort());

		try { nbytes = udpSocket.recv(message_buffer, from); }
		catch (const NetException& e) {
			RDK_ERROR_STREAM("Failure in receiving: " << e.fullwhat());
		}

		RNetMessage* message = 0;
		if (nbytes) {
			try { message = dynamic_cast<RNetMessage*>(br.deserialize(message_buffer)); }
			catch (const ReadingException& e) {
				RDK_ERROR_PRINTF("Failure in deserializing: %s", e.what());
			}
		}

		if (message) {
			SESSION_TRY_START(session)
				if (message->getSender() == session->getRepositoryName()) {
					RDK_ERROR_PRINTF("Received a message from myself via socket (?!?)")
				}
				LDOUT("Inserting a message in the inbox.")
				session->queuePush(PROPERTY_INBOX, message);
				LDOUT("Inserted a message in the inbox")
			SESSION_END_CATCH_TERMINATE(session)
		}
	}
	delete session;
#undef LDOUT
}

void UdpManager::sender() {
#ifdef UDPMANAGER_DEBUG_SENDER
	#warning UDPMANAGER_DEBUG_SENDER enabled ( A lot of debug, not good for everyday job )
	#define LDOUT(a) { RDK_DEBUG_STREAM("sender: "a) }
#else
	#define LDOUT(a)
#endif
	RDK_INFO_PRINTF("UDP sender started");

	Session* session = senderSession;	// just a shortcut

	SESSION_TRY_START(session)
	session->queueSubscribe(PROPERTY_OUTBOX);
	session->queueSetInterests(PROPERTY_OUTBOX, new UdpSenderInterests);
	session->listen(PROPERTY_OUTBOX);
	SESSION_END_CATCH_TERMINATE(session)

	BinaryWriter bw(true);

	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)
			vector<const RDK2::Object*> toSend = session->queueFreeze(PROPERTY_OUTBOX);
			LDOUT("Outbox contains " << toSend.size() << " messages");
			for (vector<const RDK2::Object*>::iterator it = toSend.begin(); it != toSend.end(); ++it) {
				LDOUT("Message: " << (*it)->getStringForVisualization().c_str());
				const RNetMessage* msg = dynamic_cast<const RNetMessage*>(*it);
				if (!msg) {
					RDK_ERROR_PRINTF("What the hell have you put in the outbox? (class: %s)",
						(*it)->getClassName().c_str());
				}
				else {
					LDOUT("Object class is '" << msg->getPayload()->getClassName().c_str() << "'");
					try {
						string outData = bw.serialize(true, msg);
						//RDK_DEBUG_PRINTF("Size: %u", outData.size());
						if (outData.size() > UDP_MESSAGE_MAXSIZE) {
							RDK_ERROR_PRINTF("Error during message serialization (message too large: "
								"size: %d, max: %d)", outData.size(), UDP_MESSAGE_MAXSIZE);
						}
						else {
							LDOUT("Sending " << outData.size() << " bytes");
							istringstream addressees(msg->getAddressees());
							string address;

							session->lock(PROPERTY_YELLOWPAGES, WHERE("Sending messages"));
							RYellowPages* yp =
								session->getObjectAsL<RYellowPages>(PROPERTY_YELLOWPAGES);

							while (addressees >> address && ! exiting) {
								if (address == repository->getRepositoryName()) {
									LDOUT("Sending a message to myself");
									session->queuePush(PROPERTY_INBOX, msg->clone());
								}
								else {
									InetAddress destination;
									if (yp->getSocketAddress(Network::UDP_IP,
									address, destination)) {
										try {
											while (!udpSocket.send(outData,
											destination) && !exiting) {
												RDK_ERROR_STREAM("Error "
												"on socket UDP");
												udpSocket.rebind();
											}
											LDOUT("Message sent to " << address << " (" <<destination.getIPAddress()<<":"<<destination.getPort()<<")");
										}
										catch (Network::NetException& netexc) {
											RDK_ERROR_STREAM("Failure in sending: " <<
											netexc.fullwhat());
										}
									}
									else {
										RDK_ERROR_PRINTF("Unknown destination '%s'", address.c_str());
									}
								}
							}
							session->unlock(PROPERTY_YELLOWPAGES);
						}
					}
					catch (const WritingException& e) {
						RDK_ERROR_PRINTF("Error during message serialization (%s)", e.what());
					}
				}
			}
		SESSION_END_CATCH_TERMINATE(session)
	}

	SESSION_TRY_START(session)
	session->queueUnsubscribe(PROPERTY_OUTBOX);
	SESSION_END_CATCH_TERMINATE(session)

	delete session;
#undef LDOUT
}

UdpManager::~UdpManager()
{
	exiting = true;
	// the receiver thread is waiting on the socket, not on the session
	udpSocket.shutdown(UDPSocket::ALL);
	// the sender thread is waiting on the session
	senderSession->wakeUp();
	// joining threads, they also delete their sessions when exiting from the loop
	pthread_join(receiverThreadId, 0);
	pthread_join(senderThreadId, 0);
	RDK_INFO_PRINTF("UDP manager closed");
}

}} // namespaces
