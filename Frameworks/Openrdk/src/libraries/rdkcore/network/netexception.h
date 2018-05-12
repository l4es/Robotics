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

#ifndef NETWORK_NETEXCEPTION_H
#define NETWORK_NETEXCEPTION_H

#include <exception>
#include <string>

namespace Network {

class NetException : public std::exception {
	public:
		NetException( const std::string& what ) throw();
		virtual ~NetException() throw();
		
		/// Return user specified cause
		virtual const char* what() const throw();
		
		/// Return errno when the exception whas launched
		virtual int geterrno() const throw();
		
		/// Return specified cause + errno
		virtual std::string fullwhat() const throw();

	protected:
		NetException() throw();

	private:
		std::string _what;
		int _errno;
};

class TCPException : public NetException {
	public:
		TCPException( const std::string& what ) throw();
		virtual ~TCPException() throw();
		
	protected:
		TCPException() throw();
};

class UDPException : public NetException {
	public:
		UDPException( const std::string& what ) throw();
		virtual ~UDPException() throw();
		
	protected:
		UDPException() throw();
};

}

#endif
