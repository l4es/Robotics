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

#ifndef NETWORK_INETADDRESS_H
#define NETWORK_INETADDRESS_H

// Libreria ARPA ( Contiene funzioni utili come inet_addr(char* addr) )
#include <arpa/inet.h>     

#include <rdkcore/network/socketaddress.h>
#include <rdkcore/network/netexception.h>

namespace Network {

class InetAddress : public SocketAddress {
	// Constructors
	public:
		/// Create a non usefull address ( all 0 )
		InetAddress() throw();
		
		/// Create an address perfect for internet receiving on port
		InetAddress( unsigned short port ) throw();
		
		/// Create a complete address
		InetAddress( const std::string& address, unsigned short port ) throw( NetException );
		
		/// Create an address from a struct sockaddr_in
		inline InetAddress( const struct sockaddr_in& address ) throw() : SocketAddress((struct sockaddr&)address) {};
		
		InetAddress( const SocketAddress& address ) throw( NetException );

	// Getters interface
	public:
		/// Get the address used for generate the ip real address
		inline const std::string& getCreationAddress() const throw() { return creationAddress; }
		
		/// Get the ip address in dot notation
		inline std::string getIPAddress() const throw() { return inet_ntoa( ((struct sockaddr_in&)address).sin_addr ); }
		
		/// Get the port
		inline unsigned short getPort() const throw() { return ntohs( ((struct sockaddr_in&)address).sin_port); }
		
		/// Get the family of the address
		inline short getFamily() const throw() { return ((struct sockaddr_in&)address).sin_family; }
		
		/// Get the struct sockaddr_in ( const version )
		inline const struct sockaddr_in& getAddress() const throw() { return (struct sockaddr_in&)address; }
		
		/// Get the struct sockaddr_in
		inline struct sockaddr_in& getAddress() throw() { return (struct sockaddr_in&)address; }
		
	// Setters interface
	public:
		/// Set a new address
		void setAddress( const std::string& address ) throw ( NetException );
		
		/// Set a new port
		inline void setPort( unsigned short port ) throw() { ((struct sockaddr_in&)address).sin_port = htons(port); }
		
		/// Set a new address from a struct sockaddr_in
		inline void setAddress( const struct sockaddr_in& address ) throw() {
			this->address = (struct sockaddr&)address; creationAddress = ""; }
			
		bool operator<( const InetAddress& ) const;
		
	private:
		std::string creationAddress;
};

}

#endif
