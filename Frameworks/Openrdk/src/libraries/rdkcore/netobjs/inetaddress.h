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

#ifndef NETOBJS_INETADDRESS_H
#define NETOBJS_INETADDRESS_H

#include "socketaddress.h"

#include <arpa/inet.h>     
#include <string>
#include <sstream>

namespace RDK2 { namespace NetObjs {

class InetAddress : public SocketAddress {
public:
/// Constructors
//@{
	/// Creates a non usable address (all 0)
	InetAddress();
		
	/// Creates an address perfect for internet receiving on port
	InetAddress(unsigned short port);
		
	/// Creates a complete address
	InetAddress(const std::string& address, unsigned short port);
		
	/// Creates an address from a struct sockaddr
	inline InetAddress(const struct sockaddr& address) : SocketAddress(address) { }
	
	/// Creates an address from a struct sockaddr_in
	inline InetAddress(const struct sockaddr_in& address) : SocketAddress((struct sockaddr&)address) { }

	/// Casts from SocketAddress
	InetAddress(const SocketAddress& address);
//@}

/// Inspection functions
//@{
	/// Returns true if this InetAddress is valid
	inline bool isValid() { return valid; }

	/// Returns the last error that prevented this address to be created (this will be set if valid is false)
	inline std::string getLastError() { return lastError; }
//@}

/// Getters
//@{
	/// Gets the string used to generate this address
	inline const std::string getCreationAddress() const { return creationAddress; }
		
	/// Gets the ip address in dot notation
	inline std::string getIPAddress() const { return inet_ntoa( ((struct sockaddr_in&)address).sin_addr ); }
		
	/// Gets the port
	inline unsigned short getPort() const { return ntohs( ((struct sockaddr_in&)address).sin_port); }
		
	/// Gets the family of the address
	inline short getFamily() const { return ((struct sockaddr_in&)address).sin_family; }
		
	/// Gets the struct sockaddr_in
	inline struct sockaddr_in getAddress() const { return (struct sockaddr_in&)address; }

	/// Gets a string representation of the object
	inline std::string toString() const { std::ostringstream oss; oss << getIPAddress() << ":" << getPort(); return oss.str(); }
//@}
			
/// Setters
//@{	
	/// Sets a new address
	void setAddress(const std::string& address);
		
	/// Sets a new port
	inline void setPort(unsigned short port) { ((struct sockaddr_in&)address).sin_port = htons(port); }
		
	/// Sets a new address from a struct sockaddr
	inline void setAddress(const struct sockaddr& address) { this->address = address; creationAddress = ""; }

	/// Sets a new address from a struct sockaddr_in
	inline void setAddress(const struct sockaddr_in& address) { this->address = (struct sockaddr&)address; creationAddress = ""; }
	
	/// Use this function only implicitly when putting InetAddress inside a std::map or similar objects			
	bool operator<(const InetAddress&) const;
//@}
		
private:
	std::string creationAddress;
	bool valid;
	std::string lastError;
};

}} // namespace

#endif
