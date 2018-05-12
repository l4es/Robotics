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

#include <fstream>
#include "ryellowpages.h"
#include <rdkcore/network/inetaddress.h>

#include <rdkcore/logging/logging.h>
#include <climits>
#define LOGGING_MODULE "RYellowPages"

//#define _DEBUG

namespace RDK2 { namespace RNetObjects {

using namespace std;


bool RYellowPages::newHost( Network::NetProtocol socket, const std::string& name, const Network::SocketAddress &address ) {
	
	std::pair< std::map< std::string, Network::SocketAddress >::iterator, bool > insert_result =
		hosts[socket].insert( std::pair< std::string, Network::SocketAddress >( name, address ) );
	
	return insert_result.second;
}

bool RYellowPages::eraseHost( Network::NetProtocol socket, const std::string& name ) {
	return hosts[socket].erase( name );
}

bool RYellowPages::loadFromFile(const std::string& nomefile)
{	
	ifstream ifs( nomefile.c_str() );
	if ( !ifs.is_open() )  {
		RDK_ERROR_STREAM( "File " << nomefile << " not found, unable to load RYellowPages data" );
		return false;
	}
	
	string name, socket, host;
	
	int port;
	
	while ( ifs >> name >> socket >> host >> port ) {
		if (name.size() > 0 && name[0] != '#') {
			try {
				Network::NetProtocol socketType = Network::stringToNetProtocol(socket);
				hosts[socketType].insert(make_pair(name, Network::InetAddress(host, (unsigned short) port)));
/*				if ( socket == "UDP_IP" )
					hosts[Network::UDP_IP].insert(
						std::pair< std::string, Network::SocketAddress >(
							name,
							Network::InetAddress( host, (unsigned short)port ) ) );
				if ( socket == "TCP_IP" )
					hosts[Network::TCP_IP].insert(
						std::pair< std::string, Network::SocketAddress >(
							name,
							Network::InetAddress( host, (unsigned short)port ) ) );
				}*/
			}
			catch ( Network::NetException& exc ) {
				RDK_ERROR_STREAM( "Error on insert for: " << name << " "
					<< socket << " " << host << " " << port );
				RDK_ERROR_STREAM( "Exception: " << exc.fullwhat() );
			}
		}
		ifs.ignore ( INT_MAX, '\n' );
	}
	
	int elements = 0;
	for ( std::map<Network::NetProtocol,std::map<std::string,Network::SocketAddress> >::iterator typeIter=hosts.begin();
		typeIter != hosts.end();
		typeIter++ )
		elements += typeIter->second.size();
	
	RDK_INFO_STREAM("Loaded YellowPages from files ( " << elements << " elements )");
	return true;
}

bool RYellowPages::getSocketAddress(	Network::NetProtocol socket,
					const std::string& name,
					Network::SocketAddress &address ) const {
	std::map< Network::NetProtocol, std::map< std::string, Network::SocketAddress > >::const_iterator
		socketFind = hosts.find( socket );
	if ( socketFind != hosts.end() ) {
		std::map< std::string, Network::SocketAddress >::const_iterator find_result = socketFind->second.find( name );
		if ( find_result != socketFind->second.end() ) {
			address = find_result->second;
			return true;
		}
	}
	return false;
}

const std::map< Network::NetProtocol, std::map< std::string, Network::SocketAddress > >& RYellowPages::getMap() const {
	return hosts;
}

}} // namespaces

