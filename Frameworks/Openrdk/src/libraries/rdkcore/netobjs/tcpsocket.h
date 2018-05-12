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

#ifndef NETOBJS_TCPSOCKET_H
#define NETOBJS_TCPSOCKET_H

#include "inetaddress.h"

#include <iostream>
#include <cstdio>

namespace RDK2 { namespace NetObjs {

class TcpSocket : protected std::streambuf, public std::iostream {
public:
/// *structors
//@{
	/// Constructs an unbound socket
	TcpSocket();
	
	/// Constructs a socket and binds it at @a address:port
	TcpSocket(const std::string& address, unsigned short port);

	/// Constructs a socket and binds it at @a address	
	TcpSocket(const InetAddress& address);

	/// Destructor		
	virtual ~TcpSocket();
//@}

/// Getters
//@{
	/// Gets the socket descriptor
	inline int getSocket() { return socket; }
	
	/// Gets the address of the peer socket
	InetAddress getPeerAddress() const { return peerAddress; }

	/// Returns true if the socket is connected.
	/** Please note that BSD socket implementation allows to know if a socket is connected only
	  * when you try to read or write to the socket.
	  */
	inline bool isConnected() const { return connected; }

	/// Returns the last error description
	inline std::string getLastError() const { return lastError; }
//@}

/// Actions on the socket
//@{
	// Disconnects and closes the socket
	void disconnect();

	// Connects to the specified @a address:port
	inline bool connect(const std::string& address, unsigned short port) { return connect(InetAddress(address, port)); }

	// Connects to the specified @a address
	bool connect(const InetAddress& address);	
//@}

private:
	friend class TcpServerSocket;

	// NOTE: only TcpServerSocket will call this
	TcpSocket(int socket, const InetAddress& caller_address);

	bool connect();

	// NOTE: the socket cannot be cloned
	TcpSocket(const TcpSocket&);
	const TcpSocket& operator=(const TcpSocket&) { return *this; }

protected:
	// Streambuffer implementation
	virtual int overflow(int c = EOF);
	virtual int underflow();
	virtual int sync();

private:
	volatile int socket;
	InetAddress peerAddress;
	char* inbuffer;
	char* outbuffer;
	const std::streamsize buffersize;
	bool connected;
	std::string lastError;
};

}}

#endif
