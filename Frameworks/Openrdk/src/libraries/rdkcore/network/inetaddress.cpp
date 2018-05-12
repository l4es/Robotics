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

#include <netdb.h>
#include <cstring>
#include <sstream>

#include <rdkcore/network/inetaddress.h>

//#define INET_ADDRESS_DEBUG

#ifdef INET_ADDRESS_DEBUG
	#include <iostream>
	#define DEBUG_STREAM(a) std::cout<<"DEBUG>>Network::inetaddress | "<<a<<std::endl;
#else
	#define DEBUG_STREAM(a)
#endif

extern int h_errno;

Network::InetAddress::InetAddress() throw() {
	((struct sockaddr_in&)address).sin_family = AF_INET;
	((struct sockaddr_in&)address).sin_port = 0;
	((struct sockaddr_in&)address).sin_addr.s_addr = INADDR_ANY;
	bzero( &(((struct sockaddr_in&)address).sin_zero), 8 );
}

Network::InetAddress::InetAddress( unsigned short port ) throw() {
	((struct sockaddr_in&)address).sin_family = AF_INET;
	((struct sockaddr_in&)address).sin_port = htons( port );
	((struct sockaddr_in&)address).sin_addr.s_addr = INADDR_ANY;
	bzero( &(((struct sockaddr_in&)address).sin_zero), 8 );
}

Network::InetAddress::InetAddress( const std::string& address, unsigned short port ) throw( NetException ) {
	this->setAddress( address );
	((struct sockaddr_in&)this->address).sin_family = AF_INET;
	((struct sockaddr_in&)this->address).sin_port = htons( port );
	creationAddress = address;
	bzero( &(((struct sockaddr_in&)this->address).sin_zero), 8 );
}

Network::InetAddress::InetAddress( const SocketAddress& address ) throw( NetException ) :
	SocketAddress( address ) {
	if ( this->address.sa_family != AF_INET )
		throw NetException("Invalid conversion from a not AF_INET address");
}



void Network::InetAddress::setAddress( const std::string& address ) throw ( NetException ) {
	if ( !inet_aton( address.c_str(), &((struct sockaddr_in&)this->address).sin_addr ) ) {
		DEBUG_STREAM("Solving IP by DNS");
		struct hostent *host = gethostbyname( address.c_str() );
		if( host ) {
			memcpy(&((struct sockaddr_in&)this->address).sin_addr.s_addr, host->h_addr_list[0], host->h_length);
		} else {
			std::ostringstream oss;
			oss<<"Unable to resolve address: "<<address<<" ( ";
			switch ( h_errno ) {
				case HOST_NOT_FOUND: oss<<"host not found"; break;
				case NO_ADDRESS: oss<<"no address"; break;
				case NO_RECOVERY: oss<<"no recovery"; break;
				case TRY_AGAIN: oss<<"try again later"; break;
				default: oss<<"unknown error type"; break;
			}
			oss<<" ) ";
			throw NetException( oss.str() );
		}
	}
}

bool Network::InetAddress::operator<( const InetAddress& addr ) const {
	if ( ((struct sockaddr_in&)address).sin_addr.s_addr <
		((struct sockaddr_in&)addr.address).sin_addr.s_addr )
		return true;
	if ( ((struct sockaddr_in&)address).sin_port <
		((struct sockaddr_in&)addr.address).sin_port )
		return true;
	return false;
}
