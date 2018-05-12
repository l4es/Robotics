/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi, Alberto Ingenito (<first_name>.<last_name>@dis.uniroma1.it)
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

#ifndef NETWORK_NETTYPES
#define NETWORK_NETTYPES

#include <string>

namespace Network {

using namespace std;

enum NetProtocol { NETPROTOCOL_ANY = 0, NETPROTOCOL_UNKNOWN = NETPROTOCOL_ANY,
	TCP_IP = 1, UDP_IP = 2, USAR_WC = 3 };
	
enum ConnectionStatus { NOT_CONNECTED = 0, CONNECTED = 1, 
	TRYING = 2, FAILED = 3, CONNECTION_RESET = 4 };

string netProtocolToString(NetProtocol netProtocol);
NetProtocol stringToNetProtocol(const string& s);

string connectionStatusToString(ConnectionStatus status);
ConnectionStatus stringToConnectionStatus(const string& s);

} // namespace

#endif
