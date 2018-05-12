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

#include <sstream>
#include <rdkcore/network/tcpsocket.h>
#include <unistd.h>
#include <cstring>

//#define TCPDEBUG
#ifdef TCPDEBUG
	#include <iostream>
	#include <iomanip>
	#define DOUT(a) std::cout<<std::setw(4)<<__LINE__<<"|"<<std::setw(15)<<__func__<<"| "<<a<<std::endl;
	#define DWRITE(a, n) std::cout<<std::setw(4)<<__LINE__<<"|"<<std::setw(15)<<__func__<<"| "; std::cout.write(a, n); std::cout<<std::endl;
#else
	#define DOUT(a)
	#define DWRITE(a, n)
#endif

//#define TCPRECVDEBUG
#ifdef TCPRECVDEBUG
	#include <iostream>
	#include <iomanip>
	#define RDOUT(a) std::cout<<std::setw(4)<<__LINE__<<"|"<<std::setw(15)<<__func__<<"| "<<a<<std::endl;
#else
	#define RDOUT(a)
#endif

//#define TCPSNIFFER
#ifdef TCPSNIFFER
	#include <iostream>
	#include <sys/time.h>
	#include <iomanip>
	#include <time.h>
	#define SNIFFWRITE(type, data, size) { timeval timestamp; gettimeofday( &timestamp, NULL ); \
					       tm time_parsed; localtime_r( (time_t*)&timestamp, &time_parsed ); \
					       std::cout<< std::setw(2) << time_parsed.tm_hour << ":" << \
					       std::setw(2) << time_parsed.tm_min << ":" << \
					       std::setw(2) << time_parsed.tm_sec << " (" << \
					       std::setw(6) << timestamp.tv_usec   << ") | " << \
					       type << " | "; std::cout.write( data, size); \
					       std::cout<<std::endl; }
#else
	#define SNIFFWRITE(type, data, size)
#endif


namespace Network {

TCPSocket::TCPSocket( std::streamsize buffersize ) throw() :
	std::streambuf(), std::iostream(this), socket( -1 ), buffersize(buffersize) {
	DOUT( "start" )
	inbuffer = new char[buffersize];
	outbuffer = new char[buffersize];
	setg ( inbuffer, inbuffer + buffersize, inbuffer + buffersize );
	setp ( outbuffer, outbuffer + buffersize );
}

TCPSocket::TCPSocket( const std::string& address, unsigned short port, std::streamsize buffersize )
	throw( TCPException, NetException ) :
	std::streambuf(), std::iostream(this), socket( -1 ), address( address, port ), buffersize(buffersize) { 
	DOUT( "start" )
	inbuffer = new char[buffersize];
	outbuffer = new char[buffersize];
	setg ( inbuffer, inbuffer + buffersize, inbuffer + buffersize );
	setp ( outbuffer, outbuffer + buffersize );
	connect();
}

TCPSocket::TCPSocket( const InetAddress& address, std::streamsize buffersize ) throw( TCPException ) :
	std::streambuf(), std::iostream(this), socket( -1 ), address( address ), buffersize(buffersize) {
	DOUT( "start" )
	inbuffer = new char[buffersize];
	outbuffer = new char[buffersize];
	setg ( inbuffer, inbuffer + buffersize, inbuffer + buffersize );
	setp ( outbuffer, outbuffer + buffersize );
	connect();
}
	
TCPSocket::~TCPSocket() {
	DOUT( "start" )
	disconnect();
	delete[] inbuffer;
	delete[] outbuffer;
	if ( socket >= 0 ) {
		flush();
		::shutdown( socket, SHUT_RDWR );
		close( socket );
	}
}

void TCPSocket::disconnect() throw( TCPException ) { 
	DOUT( "start" )
	if ( socket >= 0 ) {
		flush();
	}
	::shutdown( socket, SHUT_RDWR );
}

void TCPSocket::connect( const std::string& address, unsigned short port ) throw( TCPException, NetException ) {
	DOUT( "start" )
	disconnect();
	this->address = InetAddress( address, port );
	connect();
}

void TCPSocket::connect( const InetAddress& address ) throw( TCPException ) {
	DOUT( "start" )
	disconnect();
	this->address = address;
	connect();
}

TCPSocket::TCPSocket( int socket, const InetAddress& caller_address, std::streamsize buffersize ) : 
	std::streambuf(), std::iostream(this), socket(socket), address(caller_address), buffersize(buffersize) {
	DOUT( "start" )
	inbuffer = new char[buffersize];
	outbuffer = new char[buffersize];
	setg ( inbuffer, inbuffer + buffersize, inbuffer + buffersize );
	setp ( outbuffer, outbuffer + buffersize );
}

void TCPSocket::connect() throw( TCPException ) {
	DOUT( "start" )
	if ( socket >= 0 )
		close(socket);
	if ( ( socket = ::socket(AF_INET, SOCK_STREAM, 0) ) == -1)
		throw TCPException( "Unable to create TCP Socket" );
	
	if ( ::connect( socket, (struct sockaddr *)&address.getAddress(), sizeof( struct sockaddr ) ) == -1 ) {
		::shutdown( socket, SHUT_RDWR );
		std::ostringstream err;
		err<<"Unable to connect to the host "<<address.getIPAddress()<<":"<<address.getPort();
		throw TCPException( err.str() );
	}

	clear();
	setg ( inbuffer, inbuffer + buffersize, inbuffer + buffersize );
	setp ( outbuffer, outbuffer + buffersize );
}

int TCPSocket::overflow ( int c ) {
	DOUT( "start" )
	// If there aren't buffer return EOF
	if( !pbase() )
		throw NetException( "Buffer non consistent");

	std::streamsize sentsize;
	
	// Calculate the send size
	std::streamsize sendsize = pptr() - pbase();
	
	DOUT( "Sending "<<sendsize<<" byte" );
	DWRITE( pbase(), sendsize );
	
	// Send
	if ( sendsize ) {
		int flags = 0;
		#ifdef LINUX
		flags = MSG_NOSIGNAL;
		#elif MACOSX
		flags = SO_NOSIGPIPE;
		#endif
		sentsize = ::send( socket, (const char *)pbase(), sendsize, flags);
		
		SNIFFWRITE( "output", pbase(), sentsize )
		
		if ( sentsize < 0 )
			throw NetException( "Unable to send");
		else if ( sentsize == 0 )
			return EOF;
		else
			sendsize -= sentsize;
	}

	// Rebuffer data not sent
	if ( sendsize )
		memmove( outbuffer, outbuffer + sentsize, sendsize );

	setp( outbuffer, outbuffer + buffersize );
	pbump( sendsize );

	if( c != EOF) {
		*pptr() = (unsigned char)c;
		pbump(1);
	}
	
	return c;
}

int TCPSocket::underflow ( ) {
	DOUT( "start ( socket = " << socket << " )" )
	std::streamsize inbufferdatasize = egptr() - gptr();

	if ( inbufferdatasize ) {
		DOUT("Some data not read in the buffer inbuffer = "<<(int)inbuffer<<", gptr() = "<<(int)gptr()<<
			", inbufferdatasize = "<<inbufferdatasize);
		memmove( inbuffer, gptr(), inbufferdatasize );
	}
	
	std::streamsize torecvsize = buffersize - inbufferdatasize;
	
	std::streamsize recvsize = ::recv( socket, inbuffer + inbufferdatasize, torecvsize, 0);
	
	DOUT("Received "<<recvsize<<" bytes");
	DWRITE( inbuffer + inbufferdatasize, recvsize );
	
	if ( recvsize < 0 ) {
		std::ostringstream err;
		err<<"Unable to receive ( recv returned "<<recvsize<<" )";
		RDOUT( "Received a packet of size under 0 ( " << recvsize << " ) from " << address.getIPAddress() )
		//throw NetException( err.str() );
		return EOF;
	} else if ( recvsize == 0 ) {
		RDOUT( "Received a packet of size 0 from " << address.getIPAddress() )
		DOUT("Connection closed");
		return EOF;
	}
	SNIFFWRITE( " input", inbuffer + inbufferdatasize, recvsize )
	setg( inbuffer, inbuffer, inbuffer + inbufferdatasize + recvsize );
	
 	return (unsigned char) *gptr();
}

int TCPSocket::sync ( ) {
	DOUT( "start" )
	overflow(EOF);
	return 0;
}

InetAddress TCPSocket::getPeerAddress() const
{
	struct sockaddr sa;
	socklen_t sal = sizeof(sa);
	getpeername(socket, &sa, &sal);
	return InetAddress(sa);	 
}

}
