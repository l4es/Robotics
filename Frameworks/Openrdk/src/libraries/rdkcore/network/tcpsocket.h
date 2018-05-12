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

#ifndef NETWORK_TCPSOCKET_H
#define NETWORK_TCPSOCKET_H

#include <iostream>
#include <cstdio>

#include <rdkcore/network/netexception.h>
#include <rdkcore/network/inetaddress.h>

namespace Network {

class TCPSocket : protected std::streambuf, public std::iostream {
public:
	TCPSocket( std::streamsize buffersize = 2048 ) throw();
	
	TCPSocket( const std::string& address, unsigned short port, std::streamsize buffersize = 2048 )
		throw( TCPException, NetException );
	
	TCPSocket( const InetAddress& address, std::streamsize buffersize = 2048 ) throw( TCPException );
		
	virtual ~TCPSocket();
	
	inline int getSocket() { return socket; }

public:
	// Close the socket
	void disconnect() throw( TCPException );

	// Connect to an host
	void connect( const std::string& address, unsigned short port ) throw( TCPException, NetException );

	// Connect on a specified address
	void connect( const InetAddress& address ) throw( TCPException );
	
	const InetAddress& getAddress() const { return address; }
	InetAddress getPeerAddress() const;

private:
	friend class TCPServerSocket;
	TCPSocket( int socket, const InetAddress& caller_address, std::streamsize buffersize = 2048 );
	void connect() throw( TCPException );
	
	TCPSocket( const TCPSocket& );
	const TCPSocket& operator= ( const TCPSocket& ) { return *this; }

// Streambuffer implementation
protected:
	virtual int overflow( int c = EOF );
	virtual int underflow();
	virtual int sync();

private:
	int		socket;
	InetAddress	address;
	char*		inbuffer;
	char*		outbuffer;
	const std::streamsize buffersize;
};

}

#endif
