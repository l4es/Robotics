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

#ifndef NETWORK_TCPSERVERSOCKET_H
#define NETWORK_TCPSERVERSOCKET_H

#include <rdkcore/network/netexception.h>
#include <rdkcore/network/tcpsocket.h>
#include <rdkcore/network/inetaddress.h>
#include <unistd.h>

namespace Network {

class TCPServerSocket {
	public:
		TCPServerSocket() throw();
		TCPServerSocket( unsigned short port, unsigned short backlog = 10 ) throw( TCPException, NetException );
		TCPServerSocket( const InetAddress& address, unsigned short backlog = 10 ) throw ( TCPException );

		~TCPServerSocket() throw();

		void unbind() throw();
		inline void rebind() throw( TCPException ) { unbind(); bind(); }
		void bind( unsigned short port, unsigned short backlog = 10 ) throw( TCPException, NetException );
		void bind( const InetAddress& address, unsigned short backlog = 10 ) throw ( TCPException );
		
		TCPSocket* accept() throw( TCPException );
		
		inline const InetAddress& getAddress() { return address; }

	private:
		void bind() throw( TCPException );
	
	private:
		int socket;
		unsigned short backlog;
		InetAddress address;
};

}

#endif
