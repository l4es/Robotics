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
#include <cstring>
#include <sstream>

#include "netexception.h"

namespace Network {

NetException::NetException( const std::string& what ) throw() :
	_what( what ), _errno( errno ) {}
	
NetException::~NetException() throw() {}

const char* NetException::what() const throw() {
	return _what.c_str();
}

int NetException::geterrno() const throw() {
	return _errno;
}

std::string NetException::fullwhat() const throw() {
	if ( !_errno )
		return _what;
	else {
		std::ostringstream oss;
		oss<<_what<<" ( "<<strerror( _errno )<<" )";
		return oss.str();
	}
}

NetException::NetException() throw() : _errno( errno ) {}


TCPException::TCPException( const std::string& what ) throw() :
	NetException( what ) {}

TCPException::~TCPException() throw() {}

TCPException::TCPException() throw() : NetException() {}

UDPException::UDPException( const std::string& what ) throw() :
	NetException( what ) {}

UDPException::~UDPException() throw() {}

UDPException::UDPException() throw() : NetException() {}

}
