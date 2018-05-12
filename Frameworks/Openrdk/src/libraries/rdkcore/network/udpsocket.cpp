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


#include <rdkcore/network/udpsocket.h>
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



namespace Network {

using namespace std;

UDPSocket::UDPSocket() throw() :
	buffersize(UDP_SOCKET_DEFAULT_BUFFER_SIZE), address(), socket(-1)
{
	buffer = new char[UDP_SOCKET_DEFAULT_BUFFER_SIZE];
	rebind();
	DOUT("UDPSocket::UDPSocket( unsigned short buffersize )");
	DOUT("socket = "<<socket);
	DOUT("buffersize = "<<UDP_SOCKET_DEFAULT_BUFFER_SIZE);
}

UDPSocket::UDPSocket(unsigned short port) throw (UDPException, NetException) :
	buffersize(UDP_SOCKET_DEFAULT_BUFFER_SIZE), address(port), socket(-1)
{	
	buffer = new char[UDP_SOCKET_DEFAULT_BUFFER_SIZE];
	rebind();
	DOUT("UDPSocket::UDPSocket( unsigned short port, unsigned short buffersize )");
	DOUT("socket = "<<socket);
	DOUT("buffersize = "<<UDP_SOCKET_DEFAULT_BUFFER_SIZE);
	DOUT("port = "<<address.getPort());
}

UDPSocket::UDPSocket(const InetAddress& address) throw (UDPException) :
	buffersize(UDP_SOCKET_DEFAULT_BUFFER_SIZE), address(address), socket(-1) 
{
	buffer = new char[UDP_SOCKET_DEFAULT_BUFFER_SIZE];
	rebind();
	DOUT("UDPSocket::UDPSocket( const InetAddress& address, unsigned short buffersize )");
	DOUT("socket = "<<socket);
	DOUT("buffersize = "<<UDP_SOCKET_DEFAULT_BUFFER_SIZE);
	DOUT("port = "<<address.getPort());
}

UDPSocket::~UDPSocket() {
	delete[] buffer;
	if ( socket >= 0 ) {
		shutdown(ALL);
		close(socket);
	}
}

bool UDPSocket::isBound() throw() {
	return socket >= 0;
}

void UDPSocket::unbind() throw() {
	shutdown(ALL);
	if (socket >= 0) {
		close(socket);
		socket = -1;
	}
}

void UDPSocket::rebind() throw( UDPException ) 
{
	unbind();
	DOUT("Socket binding");
	DOUT("Socket = "<<socket);
	DOUT("Bind: " << address.getIPAddress());
	DOUT("Port = "<<address.getPort());
	if ( socket < 0 )
		if ( ( socket = ::socket(AF_INET, SOCK_DGRAM, 0) ) < 0 )
			throw UDPException("Unable obtain a UDP socket, stop working");

	if ( ::bind( socket, (struct sockaddr *)&address.getAddress(), sizeof(struct sockaddr) ) < 0 ) {
		shutdown(ALL);
		std::ostringstream err;
		err<<"Unable to bind UDP socket to address " << address.getIPAddress()
			<< ", port " << address.getPort();
		throw UDPException( err.str() );
	}
	
	// Set BROADCAST ACTIVE
	int on=1;
	setsockopt(socket, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
	DOUT("Socket bound");
	DOUT("Socket = "<<socket);
	DOUT("Bind: " << address.getIPAddress());
	DOUT("Port = "<<address.getPort());
}

void UDPSocket::bind( unsigned short port ) throw( UDPException, NetException ) {
	unbind();
	address = InetAddress( port );
	rebind();
}

void UDPSocket::bind( const InetAddress& address ) throw( UDPException ) {
	unbind();
	this->address = address;
	rebind();
}

void UDPSocket::newBuffer( unsigned short buffersize ) throw() {
	delete buffer;
	this->buffersize = buffersize;
	buffer = new char[buffersize];
}

unsigned short UDPSocket::send( const std::string& data, const InetAddress& address ) throw ( UDPException ) {
	if ( !data.size() ) {
		std::cout<<__FILE__<<":"<<__FUNCTION__<<": You cannot send empty udp packet!!! "
			"( An empty packet is the shutdown declaration)\n";
		return 0;
	} else if ( data.size() > buffersize ) {
		throw UDPException( "Message too big for the buffer!!!" );
	}
	
	int bytesent;
	DOUT( "socket    = "<<socket );
	DOUT( "data      = "<<data );
	DOUT( "data size = "<<data.size() ); 
	DOUT( "IP        = "<<address.getIPAddress());
	
	if ( ( bytesent = sendto(	socket,						// Socket
					data.data(),					// Message
					data.size(),					// Message lenght
					0,						// Send flags
					(struct sockaddr *)&address.getAddress(),	// Destination address
					sizeof( struct sockaddr ) ) )			// Lenght of destination address
					== -1 ) {
		std::ostringstream err;
		err<<"Send failed to "<<address.getIPAddress()<<":"<< address.getPort();
		throw UDPException( err.str() );
	}
	DOUT( "byte sent = "<<bytesent );
	return bytesent;
}

unsigned short UDPSocket::recv( std::string& data, InetAddress& address, double timeoutSecs ) throw ( UDPException ) {
	RECVDOUT( "socket     = "<<socket );
	RECVDOUT( "&address   = "<<(std::hex)<<&address<<(std::dec));
	IPERDOUT( "myaddr     = "<<address.getIPAddress() );
	
	socklen_t caller_address_lenght = sizeof(struct sockaddr_in);
	
//	std::cout<<"buffersize = " << buffersize << std::endl;
	
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
	if (canRead) bytes_received = recvfrom(	socket,
					buffer,
					buffersize,
					0,
					(struct sockaddr *)&address.getAddress(),
					&caller_address_lenght );

// 	for (size_t i = 0; i < bytes_received; i++) {
// 		ostringstream sas;
// 		sas << i << ":" << (int) buffer[i] << " (" << buffer[i] << ")   ";
// 		cout << sas.str() << endl;
// 	}

//	std::cout<<"bytes_received = " << bytes_received << std::endl;
					
	RECVDOUT("bytes received: "<<bytes_received);
	if ( bytes_received < 0 ) {
		std::ostringstream err;
		err<<"Receive failed ( receiving from port "<<this->address.getPort()<<" ).";
		RECVDOUT( "Failed IP:      "<<address.getIPAddress() );
		RECVDOUT( "Failed Port:    "<<address.getPort() );
		throw UDPException( err.str() );
	} else {
		RECVDOUT( "IP:            "<<address.getIPAddress() );
		RECVDOUT( "Port:           "<<address.getPort() );
		data.clear();
		data.append( buffer, bytes_received );
		return bytes_received;
	}
}

bool UDPSocket::shutdown( ShutdownType  how ) {
	return ( ::shutdown( socket, how ) != -1 );
}

}
