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

#include "tcpserversocket.h"

#include <errno.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sstream>

#define DEFAULT_BACKLOG 10

namespace RDK2 { namespace NetObjs {

TcpServerSocket::TcpServerSocket() : socket(-1), backlog(DEFAULT_BACKLOG) { }

TcpServerSocket::~TcpServerSocket()
{
	unbind();
}

void TcpServerSocket::unbind() {
	if (socket >= 0) {
		::shutdown(socket, SHUT_RDWR);
		close(socket);
		socket = -1;
	}
}

bool TcpServerSocket::bind(unsigned short port)
{
	unbind();
	address = InetAddress(port);
	return bind();
}

bool TcpServerSocket::bind(const InetAddress& address)
{	
	unbind();	
	this->address = address;
	return bind();
}

TcpSocket* TcpServerSocket::accept()
{
	int newsocket;
	socklen_t addr_len_call = sizeof(struct sockaddr_in);

	struct sockaddr sa;
	if ((newsocket = ::accept(socket, &sa, &addr_len_call)) == -1) {
		lastError = "Unable to accept an extern connection";
		return 0;
	}
	
	return new TcpSocket(newsocket, InetAddress(sa));
}

bool TcpServerSocket::bind()
{
	// Get a socket from kernel
	if (socket < 0) {
		if ((socket = ::socket(AF_INET, SOCK_STREAM, 0)) == -1) {
			lastError = "Unable to open an AF_INET stream socket";
			return false;
		}
	}

	// Avoid that damned "port already open" if the process dies without shutdown and close on the listening socket
	int reuseAddr = 1;
	if (setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, &reuseAddr, sizeof(reuseAddr))) {
		unbind();
		lastError = "Unable to set SO_REUSEADDR on the listening socket";
		return false;
	}
	
	// Bind the socket to the address/port
	struct sockaddr_in sa = address.getAddress();
	if (::bind(socket, (const struct sockaddr*)&sa, sizeof(struct sockaddr))) {
		unbind();
		std::ostringstream oss;
		oss << "Unable to bind AF_INET server socket to address: " << address.getIPAddress()<<":"<<address.getPort();
		lastError = oss.str();
		return false;
	}

	// Attiva l'ascolto sulla porta
	if (listen(socket, backlog) == -1) {
		unbind();
		std::ostringstream oss;
		oss << "Unable to listen AF_INET stream socket to address: " << address.getIPAddress()<<":"<<address.getPort();
		lastError = oss.str();
		return false;
	}

	return true;
}

}}
