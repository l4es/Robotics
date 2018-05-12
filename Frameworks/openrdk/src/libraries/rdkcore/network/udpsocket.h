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

#ifndef NETWORK_UDPSOCKET_H
#define NETWORK_UDPSOCKET_H

#include <rdkcore/network/netexception.h>
#include <rdkcore/network/inetaddress.h>

#define UDP_SOCKET_DEFAULT_BUFFER_SIZE 1500

namespace Network {

class UDPSocket {
	// Constructors
	public:
		UDPSocket() throw();
		UDPSocket(unsigned short port) throw (UDPException, NetException);
		UDPSocket(const InetAddress& address) throw (UDPException);
	
		~UDPSocket();
	// Interface
	public:
		// Return true if socket is opened
		bool isBound() throw();
		
		enum ShutdownType { READ = SHUT_RD, WRITE = SHUT_WR, ALL = SHUT_RDWR };
		bool shutdown( ShutdownType  how );
		
		// Close the socket
		void unbind() throw();
		
		// Open ( if needed close the socket before ) on the last address
		void rebind() throw( UDPException );
		
		// Bind on a new port
		void bind( unsigned short port ) throw( UDPException, NetException );

		// Bind on a specified address
		void bind( const InetAddress& address ) throw( UDPException );
		
		// Change buffer size
		void newBuffer( unsigned short buffersize ) throw();
		
		int getRealAddress( InetAddress& addr ) throw() {
			socklen_t size = sizeof( struct sockaddr );
			return ::getsockname( socket, (struct sockaddr*)&addr.getAddress(), &size);
		}

		inline InetAddress getAddress() { return address; }
		
		inline int getSocket() const {return socket;}
		
		// Send a RAW packet
		unsigned short send( const std::string& data, const InetAddress& address ) throw ( UDPException );

		// Receive a RAW packet, and store the address in address
		unsigned short recv( std::string& data, InetAddress& address, double timeoutSecs = 0.0) throw ( UDPException );

		// Receive a packet without storing the address
		inline unsigned short recv(std::string& data) throw (UDPException) { InetAddress ia; return recv(data, ia); }
	protected:
		unsigned short buffersize;
		InetAddress address;
		int socket;
		char* buffer;
};

}

#endif
