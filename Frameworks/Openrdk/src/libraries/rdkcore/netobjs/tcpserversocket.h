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

#ifndef NETOBJS_TCPSERVERSOCKET_H
#define NETOBJS_TCPSERVERSOCKET_H

#include "tcpsocket.h"
#include "inetaddress.h"

#include <unistd.h>
#include <string>

namespace RDK2 { namespace NetObjs {

class TcpServerSocket {
public:
/// *structors
//@{
	/// Constructs an unbounded server socket
	TcpServerSocket();

	/// Destructor
	~TcpServerSocket();
//@}

/// Binding
//@{
	/// Binds the socket to the specified @a port
	/** @return true if the operation succeeds */
	bool bind(unsigned short port);

	/// Binds the socket to the specified @a address
	/** @return true if the operation succeeds */
	bool bind(const InetAddress& address);

	/// Unbinds the socket
	void unbind();

	/// Rebinds means to unbind() the socket and rebind to the same address
	inline bool rebind() { unbind(); return bind(); }
//@}

/// Accepting an incoming connection
//@{
	/// Accepts incoming connections from the clients. This function is blocking.
	/** If the socket is unbound during an accept blocking call, the function returns 0. */
	TcpSocket* accept();
//@}

/// Getters
//@{
	/// Returns the address of this socket
	inline InetAddress getAddress() { return address; }

	/// Returns the description of the last error occurred
	inline std::string getLastError() { return lastError; }
//@}

private:
	bool bind();
	
	volatile int socket;
	InetAddress address;
	std::string lastError;
	unsigned short backlog;
};

}}

#endif
