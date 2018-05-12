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

#include "repository.h"
#include "session.h"

#include <rdkcore/profiling/profiling.h>
#include <rdkcore/object/objectdiff.h>
#include <rdkcore/repository_struct/rpropertydef.h>
#include <rdkcore/rnetobjects/ryellowpages.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "Session"

#include <rdkcore/object/objectmanager.h>

//#define DEBUG_THIS
#ifdef DEBUG_THIS
#warning a lot of debug
#define DOUT(a, args...) RDK_DEBUG_PRINTF(a, ## args)
#else
#define DOUT(a, args...)
#endif

namespace RDK2 { namespace RepositoryNS {

using namespace RDK2::Profiling;

void Session::tryConnectTo(Network::NetProtocol protocol, const string& peerName)
{
	checkStarted();
	if (protocol == Network::TCP_IP) {
		if (!repository->tcpManager) {
			RDK_ERROR_PRINTF("Trying to TCP connect with '%s', but there's no TCP manager!", 
				peerName.c_str());
		}
		else repository->tcpManager->tryConnectTo(peerName, 0);
	}
	else if (protocol == Network::USAR_WC) {
		queuePush("/usarWCClient/in/requestQueue", new RString("CONNECT " + peerName));
	}
	else {
		RDK_ERROR_PRINTF("Unknown network protocol for connections: %d (%s)",
			protocol, Network::netProtocolToString(protocol).c_str());
	}
}

void Session::tryDisconnectFrom(Network::NetProtocol protocol, const string& peerName)
{
	checkStarted();
	if (protocol == Network::TCP_IP) {
		RDK_ERROR_PRINTF("TCP disconnect not yet implemented");
	}
	else if (protocol == Network::USAR_WC) {
		queuePush("/usarWCClient/in/requestQueue", new RString("DISCONNECT " + peerName));
	}
	else {
		RDK_ERROR_PRINTF("Unknown network protocol for connections: %d (%s)",
			protocol, Network::netProtocolToString(protocol).c_str());
	}
}

Network::ConnectionStatus Session::getConnectionStatusWith(Network::NetProtocol protocol, const string& peerName)
{
	checkStarted();
	if (protocol == Network::TCP_IP) {
		if (!repository->tcpManager) {
			RDK_ERROR_PRINTF("Trying to get TCP connection status with '%s', "
				"but there's no TCP manager!", peerName.c_str());
			return Network::NOT_CONNECTED;
		}
		else return repository->tcpManager->getConnectionStatusWith(peerName);
	}
	else if (protocol == Network::USAR_WC) {
		string s = getString("/usarWCClient/out/connectedAgents");
		vector<string> agentConns = tokenize(s, " ");
		for (size_t i = 0; i < agentConns.size(); i++) {
			vector<string> status = tokenize(agentConns[i], ":");
			if (status[0] == peerName) {
				return Network::stringToConnectionStatus(status[1]);
			}
		}
		RDK_ERROR_PRINTF("Unknown agent '%s' protocol USAR_WC", peerName.c_str());
		return Network::NOT_CONNECTED;
	}
	else {
		RDK_ERROR_PRINTF("Unknown network protocol for connections: %d (%s)",
			protocol, Network::netProtocolToString(protocol).c_str());
		return Network::NOT_CONNECTED;
	}
}

bool Session::getInetAddressOf(const string& name, Network::NetProtocol socketType, InetAddress& address) throw (SessionException)
{
	if (name.find(":") != string::npos) {
		string addr = name.substr(0, name.find(":"));
		short port = atoi(name.substr(name.find(":")+1).c_str());
		address = InetAddress(addr, port);
		return true;
	}
	else {
		bool found = false;
		lock(PROPERTY_YELLOWPAGES, HERE);
		RYellowPages* yp = getObjectAsL<RYellowPages>(PROPERTY_YELLOWPAGES);
		found = yp->getSocketAddress(socketType, name, address);
		unlock(PROPERTY_YELLOWPAGES);
		return found;
	}
}

}} // namespace
