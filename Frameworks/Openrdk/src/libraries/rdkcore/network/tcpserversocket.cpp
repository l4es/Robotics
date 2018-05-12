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

#include <errno.h>
#include <sys/socket.h>
#include <unistd.h>

#include <sstream>

#include <rdkcore/network/tcpserversocket.h>

namespace Network {

TCPServerSocket::TCPServerSocket() throw() :
	socket(-1) {
}

TCPServerSocket::TCPServerSocket( unsigned short port, unsigned short backlog ) throw( TCPException, NetException ) :
	socket(-1), address( port ) {
	
	this->backlog = backlog;
	
	bind();
}

TCPServerSocket::TCPServerSocket( const InetAddress& address, unsigned short backlog ) throw ( TCPException ) :
	socket(-1), backlog(backlog), address(address) {
	
	bind();
}

TCPServerSocket::~TCPServerSocket() throw() {
	unbind();
}

void TCPServerSocket::unbind() throw() {
	if ( socket >= 0 ) {
		::shutdown( socket, SHUT_RDWR );
		close( socket );
		socket = -1;
	}
}

void TCPServerSocket::bind( unsigned short port, unsigned short backlog ) throw( TCPException, NetException ) {
	
	unbind();
	
	this->backlog = backlog; 
	
	address = InetAddress( port );
	
	bind();
}

void TCPServerSocket::bind( const InetAddress& address, unsigned short backlog ) throw ( TCPException ) {
	
	unbind();
	
	this->backlog = backlog; 
	
	this->address = address;
	
	bind();
}

TCPSocket* TCPServerSocket::accept() throw( TCPException ) {
	// Accetta una connessione TCP
	int newsocket;
	InetAddress newaddress;
	socklen_t addr_len_call = sizeof( struct sockaddr_in );
	
	if ( ( newsocket = ::accept( socket, (struct sockaddr *)&newaddress.getAddress(), &addr_len_call)) == -1 ) {
		throw TCPException("Unable to accept an extern connection");
	}
	
	return new TCPSocket( newsocket, newaddress );
}

void TCPServerSocket::bind() throw( TCPException ) {
	// Get a socket from kernel
	if ( socket < 0 )
		if ( ( socket = ::socket(AF_INET, SOCK_STREAM, 0) ) == -1 )
			throw TCPException("Unable to open an AF_INET stream socket");

	// DC: Avoid damned "port already open" if the process dies without shutdown and close on the listening socket
	int reuseAddr = 1;
	if (setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, &reuseAddr, sizeof(reuseAddr))) {
		unbind();
		throw TCPException("Unable to set SO_REUSEADDR on the listening socket");
	}
	
	// Socket binded to the port
	if ( ::bind( socket, (struct sockaddr *)&address.getAddress(), sizeof(struct sockaddr)) ) {
		unbind();
		std::ostringstream oss;
		oss<<"Unable to bind AF_INET server socket to address: "<<address.getIPAddress()<<":"<<address.getPort();
		throw TCPException(oss.str());
	}

	// Attiva l'ascolto sulla porta
	if ( listen( socket, backlog ) == -1 ) {
		unbind();
		std::ostringstream oss;
		oss<<"Unable to listen AF_INET stream socket to address: "<<address.getIPAddress()<<":"<<address.getPort();
		throw TCPException(oss.str());
	}
}

}
