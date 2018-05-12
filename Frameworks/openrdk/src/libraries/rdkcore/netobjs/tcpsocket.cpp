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

#include "tcpsocket.h"

#include <errno.h>
#include <sstream>
#include <unistd.h>
#include <cstring>

#define DEFAULT_BUFFER_SIZE 2048

namespace RDK2 { namespace NetObjs {

TcpSocket::TcpSocket() : std::streambuf(), std::iostream(this), socket(-1), buffersize(DEFAULT_BUFFER_SIZE), connected(false)
{
	inbuffer = new char[buffersize];
	outbuffer = new char[buffersize];
	setg(inbuffer, inbuffer + buffersize, inbuffer + buffersize);
	setp(outbuffer, outbuffer + buffersize);
}

TcpSocket::~TcpSocket()
{
	disconnect();
	delete[] inbuffer;
	delete[] outbuffer;
}

void TcpSocket::disconnect()
{
	if (socket >= 0) {
		flush();
		::shutdown(socket, SHUT_RDWR);
		close(socket);
		socket = -1;
		connected = false;
	}
}

bool TcpSocket::connect(const InetAddress& address)
{
	disconnect();
	this->peerAddress = address;
	return connect();
}

TcpSocket::TcpSocket(int socket, const InetAddress& peerAddress) : 
	std::streambuf(), std::iostream(this), socket(socket), peerAddress(peerAddress), buffersize(DEFAULT_BUFFER_SIZE)
{
	inbuffer = new char[buffersize];
	outbuffer = new char[buffersize];
	setg(inbuffer, inbuffer + buffersize, inbuffer + buffersize) ;
	setp(outbuffer, outbuffer + buffersize);
	connected = true;	// the socket is already connected
}

bool TcpSocket::connect()
{
	if (socket >= 0) disconnect();
	
	if ((socket = ::socket(AF_INET, SOCK_STREAM, 0) ) == -1) {
		lastError = "Cannot create a TCP socket";
		return false;
	}

	struct sockaddr_in sa = peerAddress.getAddress();
	if (::connect(socket, (const struct sockaddr*)&sa, sizeof(struct sockaddr)) == -1) {
		::shutdown(socket, SHUT_RDWR);
		std::ostringstream err;
		err << "Unable to connect to the host " << peerAddress.getIPAddress() << ":" << peerAddress.getPort();
		lastError = err.str();
		return false;
	}

	clear();
	setg(inbuffer, inbuffer + buffersize, inbuffer + buffersize);
	setp(outbuffer, outbuffer + buffersize);

	connected = true;
	return true;
}

int TcpSocket::overflow(int c)
{
	// If there aren't buffer return EOF
	if (!pbase()) {
		lastError = "Buffer non consistent";
		connected = false;
		return 0;
	}

	std::streamsize sentsize;
	
	// Calculate the send size
	std::streamsize sendsize = pptr() - pbase();
	
	// Send
	if (sendsize) {
		int flags = 0;
		#ifdef LINUX
		flags = MSG_NOSIGNAL;
		#elif MACOSX
		flags = SO_NOSIGPIPE;
		#elif CYGWIN
		flags = MSG_NOSIGNAL;
		#else
		#warning No LINUX, MACOSX or CYGWIN defined, assuming LINUX
		flags = MSG_NOSIGNAL;
		#endif
		sentsize = ::send(socket, (const char *)pbase(), sendsize, flags);
		
		if (sentsize < 0) {
			lastError = "Unable to send";
			connected = false;
			return EOF;
		}
		else if (sentsize == 0) {
			lastError = "Write of size 0";
			connected = false;
			return EOF;
		}
		else sendsize -= sentsize;
	}

	// Rebuffer data not sent
	if (sendsize) memmove(outbuffer, outbuffer + sentsize, sendsize);

	setp(outbuffer, outbuffer + buffersize);
	pbump(sendsize);

	if (c != EOF) {
		*pptr() = (unsigned char)c;
		pbump(1);
	}
	
	return c;
}

int TcpSocket::underflow()
{
	std::streamsize inbufferdatasize = egptr() - gptr();

	if (inbufferdatasize) memmove(inbuffer, gptr(), inbufferdatasize);
	
	std::streamsize torecvsize = buffersize - inbufferdatasize;
	std::streamsize recvsize = ::recv(socket, inbuffer + inbufferdatasize, torecvsize, 0);
	
	if (recvsize < 0) {
		std::ostringstream err;
		err << "Unable to receive (recv returned " << recvsize << ")";
		lastError = err.str();
		connected = false;
		return EOF;
	}
	else if (recvsize == 0) {
		// connection closed by the peer
		connected = false;
		return EOF;
	}
	setg(inbuffer, inbuffer, inbuffer + inbufferdatasize + recvsize);
	
 	return (unsigned char) *gptr();
}

int TcpSocket::sync()
{
	overflow(EOF);
	return 0;
}

}} // namespace
