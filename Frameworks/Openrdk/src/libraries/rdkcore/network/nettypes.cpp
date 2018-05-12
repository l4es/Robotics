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

#include "nettypes.h"

namespace Network {

string netProtocolToString(NetProtocol netProtocol)
{
	switch (netProtocol) {
		case TCP_IP: return "TCP_IP";
		case UDP_IP: return "UDP_IP";
		case USAR_WC: return "USAR_WC";
		case NETPROTOCOL_UNKNOWN: return "?";
	}
	return "";
}

NetProtocol stringToNetProtocol(const string& s)
{
	if (s == "TCP_IP" || s == "TCP") return TCP_IP;
	else if (s == "UDP_IP" || s == "UDP") return UDP_IP;
	else if (s == "USAR_WC") return USAR_WC;
	return NETPROTOCOL_UNKNOWN;
}

ConnectionStatus stringToConnectionStatus(const string& s)
{
	if (s == "NOT_CONNECTED") return NOT_CONNECTED;
	else if (s == "CONNECTED") return CONNECTED;
	else if (s == "TRYING") return TRYING;
	else if (s == "FAILED") return FAILED;
	else if (s == "CONNECTION_RESET") return CONNECTION_RESET;
	else return NOT_CONNECTED;	// XXX
}

string connectionStatusToString(ConnectionStatus status)
{
	switch (status) {
		case NOT_CONNECTED: return "NOT_CONNECTED";
		case CONNECTED: return "CONNECTED";
		case TRYING: return "TRYING";
		case FAILED: return "FAILED";
		case CONNECTION_RESET: return "CONNECTION_RESET";
	}
	return "";
}

} // namespace
