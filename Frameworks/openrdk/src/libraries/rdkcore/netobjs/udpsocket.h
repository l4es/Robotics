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

#ifndef NETOBJS_UDPSOCKET_H
#define NETOBJS_UDPSOCKET_H

#include "inetaddress.h"

#define UDP_SOCKET_DEFAULT_BUFFER_SIZE 1500

namespace RDK2 { namespace NetObjs {

class UdpSocket {
public:
	/// *structors
	//@{
	UdpSocket();
	UdpSocket(unsigned short port);
	UdpSocket(const InetAddress& address);
	~UdpSocket();
	//@}

public:
	/// Returns true if socket is bound to a port
	bool isBound();
	
	/// Shuts down the socket
	/** @how can be SHUT_RD, SHUT_WR or SHUT_RDWR*/
	bool shutdown(int how);
	
	/// Unbinds the socket from the port
	void unbind();
	
	/// Rebinds the socket to the last address/port
	bool rebind();
	
	/// Binds the socket to a new port
	bool bind(unsigned short port);

	/// Binds the socket to a specified address
	bool bind(const InetAddress& address);
	
	/// Gets the address to which this socket is bound
	inline InetAddress getAddress() { return address; }
	
	/// returns the socket file descriptor, for low-level operations
	inline int getSocket() const { return socket; }
	
	/// Sends a packet packet
	ssize_t send(const std::string& data, const InetAddress& address);

	/// Receives a packet, and store the address in address
	ssize_t recv(std::string& data, InetAddress& address, double timeoutSecs = 0.0);

	/// Receives a packet without storing the address
	inline ssize_t recv(std::string& data) { InetAddress ia; return recv(data, ia); }

protected:
	// NOTE: these variables can be accessed by more than one thread (usually a thread use the socket to send, another one to receive)
	// only send(...) and recv(...) functions can be 
	unsigned short buffersize;
	InetAddress address;
	int socket;
	char* buffer;
};

}} // namespaces

#endif
