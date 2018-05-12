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


#include "udpsocket.h"
#include <errno.h>
#include <sstream>
#include <iostream>
#include <unistd.h>

//#define _DEBUG
#ifdef _DEBUG
	#define DOUT(a) std::cout<<__func__<<": "<<a<<std::endl
#else
	#define DOUT(a)
#endif

//#define RECV_DEBUG
#ifdef RECV_DEBUG
	#define RECVDOUT(a) std::cout<<__func__<<": "<<a<<std::endl
#else
	#define RECVDOUT(a)
#endif

//#define IPERSTRANGEBUG_DEBUG
#ifdef IPERSTRANGEBUG_DEBUG
	#define IPERDOUT(a) std::cout<<__func__<<": "<<a<<std::endl
#else
	#define IPERDOUT(a)
#endif

namespace RDK2 { namespace NetObjs {

using namespace std;

UdpSocket::UdpSocket() :
	buffersize(UDP_SOCKET_DEFAULT_BUFFER_SIZE), address(), socket(-1)
{
	buffer = new char[UDP_SOCKET_DEFAULT_BUFFER_SIZE];
	rebind();
	DOUT("UdpSocket::UdpSocket( unsigned short buffersize )");
	DOUT("socket = "<<socket);
	DOUT("buffersize = "<<UDP_SOCKET_DEFAULT_BUFFER_SIZE);
}

UdpSocket::UdpSocket(unsigned short port) :
	buffersize(UDP_SOCKET_DEFAULT_BUFFER_SIZE), address(port), socket(-1)
{	
	buffer = new char[UDP_SOCKET_DEFAULT_BUFFER_SIZE];
	rebind();
	DOUT("UdpSocket::UdpSocket( unsigned short port, unsigned short buffersize )");
	DOUT("socket = "<<socket);
	DOUT("buffersize = "<<UDP_SOCKET_DEFAULT_BUFFER_SIZE);
	DOUT("port = "<<address.getPort());
}

UdpSocket::UdpSocket(const InetAddress& address) :
	buffersize(UDP_SOCKET_DEFAULT_BUFFER_SIZE), address(address), socket(-1) 
{
	buffer = new char[UDP_SOCKET_DEFAULT_BUFFER_SIZE];
	rebind();
	DOUT("UdpSocket::UdpSocket( const InetAddress& address, unsigned short buffersize )");
	DOUT("socket = "<<socket);
	DOUT("buffersize = "<<UDP_SOCKET_DEFAULT_BUFFER_SIZE);
	DOUT("port = "<<address.getPort());
}

UdpSocket::~UdpSocket()
{
	delete[] buffer;
	if ( socket >= 0 ) {
		shutdown(SHUT_RDWR);
		close(socket);
	}
}

bool UdpSocket::isBound()
{
	return socket >= 0;
}

void UdpSocket::unbind()
{
	shutdown(SHUT_RDWR);
	if (socket >= 0) {
		close(socket);
		socket = -1;
	}
}

bool UdpSocket::rebind() 
{
	unbind();
	DOUT("Socket binding");
	DOUT("Socket = "<<socket);
	DOUT("Bind: " << address.getIPAddress());
	DOUT("Port = "<<address.getPort());
	if (socket < 0)
		if ((socket = ::socket(AF_INET, SOCK_DGRAM, 0)) < 0) return false;

	InetAddress ia = address.getAddress();
	if (::bind(socket, (struct sockaddr *) &ia, sizeof(struct sockaddr)) < 0) {
		shutdown(SHUT_RDWR);
		return false;
	}
	
	// Set BROADCAST ACTIVE
	int on = 1;
	setsockopt(socket, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
	DOUT("Socket bound");
	DOUT("Socket = "<<socket);
	DOUT("Bind: " << address.getIPAddress());
	DOUT("Port = "<<address.getPort());
	return true;
}

bool UdpSocket::bind(unsigned short port)
{
	unbind();
	address = InetAddress( port );
	return rebind();
}

bool UdpSocket::bind(const InetAddress& address)
{
	unbind();
	this->address = address;
	return rebind();
}

ssize_t UdpSocket::send(const std::string& data, const InetAddress& address)
{
	if (!data.size()) return 0;
	else if (data.size() > buffersize) {
		//sendReason = "Message too big for the buffer!";
		return -1;
	}
	
	DOUT( "socket    = "<<socket );
	DOUT( "data      = "<<data );
	DOUT( "data size = "<<data.size() ); 
	DOUT( "IP        = "<<address.getIPAddress());

	InetAddress ia = address.getAddress();
	int bytesent = sendto(socket, data.data(), data.size(), 0, (struct sockaddr *)&ia, sizeof(struct sockaddr));
	DOUT( "byte sent = "<<bytesent );
	return bytesent;
}

ssize_t UdpSocket::recv(std::string& data, InetAddress& address, double timeoutSecs)
{
	RECVDOUT( "socket     = "<<socket );
	RECVDOUT( "&address   = "<<(std::hex)<<&address<<(std::dec));
	IPERDOUT( "myaddr     = "<<address.getIPAddress() );
	
	
	bool canRead = true;
	
	if (timeoutSecs != 0.0) {
		fd_set rfds;
		struct timeval tv;
		FD_ZERO(&rfds);
		FD_SET(socket, &rfds);
		tv.tv_sec = 0;
		tv.tv_usec = (int) (timeoutSecs * 1000 * 1000);
		int retval = select(socket + 1, &rfds, 0, 0, &tv);
		canRead = (retval > 0);
	}
	
	int bytes_received = 0;
	struct sockaddr_in sa;
	socklen_t caller_address_lenght = sizeof(struct sockaddr_in);
	if (canRead) bytes_received = recvfrom(socket, buffer, buffersize, 0, (struct sockaddr*)&sa, &caller_address_lenght);
	address = InetAddress(sa);

	RECVDOUT("bytes received: "<<bytes_received);
	if (bytes_received < 0) {
		std::ostringstream err;
		err<<"Receive failed ( receiving from port "<<this->address.getPort()<<" ).";
		RECVDOUT( "Failed IP:      "<<address.getIPAddress() );
		RECVDOUT( "Failed Port:    "<<address.getPort() );
		return bytes_received;
	}
	else {
		RECVDOUT( "IP:            "<<address.getIPAddress() );
		RECVDOUT( "Port:           "<<address.getPort() );
		data.clear();
		data.append( buffer, bytes_received );
		return bytes_received;
	}
}

bool UdpSocket::shutdown(int how)
{
	return ::shutdown(socket, how) != -1;
}

}} // namespaces

