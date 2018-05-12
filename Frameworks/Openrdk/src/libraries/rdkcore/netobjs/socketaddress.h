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

#ifndef NETOBJS_SOCKETADDRESS_H
#define NETOBJS_SOCKETADDRESS_H

#include <sys/socket.h>

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *   INTERNET STRUCTURES MEMO
 *   struct sockaddr {
 *       unsigned short    sa_family;    // address family, AF_xxx
 *       char              sa_data[14];  // 14 bytes of protocol address
 *   }; 
 *
 *   struct sockaddr_in {
 *       short int          sin_family;  // Address family
 *       unsigned short int sin_port;    // Port number
 *       struct in_addr     sin_addr;    // Internet address
 *       unsigned char      sin_zero[8]; // Same size as struct sockaddr
 *   }; 
 *
 *  // Internet address (a structure for historical reasons)
 *   struct in_addr {
 *       unsigned long s_addr; // that's a 32-bit long, or 4 bytes
 *   }; 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

namespace RDK2 { namespace NetObjs {

class SocketAddress {
public:
/// Constructors
//@{
	/// Creates a non usable address (all 0)
	SocketAddress();
		
	/// Creates an address from a struct sockaddr
	inline SocketAddress(const struct sockaddr& address) : address(address) { }
//@}

/// Getters
//@{
	/// Gets the family of the address
	inline short getFamily() const  { return address.sa_family; }
		
	/// Gets the struct sockaddr
	inline struct sockaddr getAddress() { return address; }
//@}

protected:
	struct sockaddr address;
};

}}

#endif
