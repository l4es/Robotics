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

#ifndef RDK2_RNETOBJECTS_RYELLOWPAGES
#define RDK2_RNETOBJECTS_RYELLOWPAGES

#include <string>
#include <map>
#include <pthread.h>

#include <rdkcore/object/object.h>
#include <rdkcore/network/socketaddress.h>

#include "rnetmessage.h"

namespace RDK2 { namespace RNetObjects {

	/** Contains known hosts.
	 *
	 *  For compatibility for ALL socket type, all the data pass as sockadrr.
	 *  This class can be copied without problems, since there aren't pointer by now
	 *
	 *  /author Alberto Ingenito (alberto.ing@gmail.com)
	 */
	class RYellowPages : public RDK2::Object
	{
		/// Host list.
		std::map<Network::NetProtocol,
			std::map< std::string, Network::SocketAddress > > hosts;

		public:
			RDK2_DEFAULT_CLONE(RYellowPages);

			typedef std::map<Network::NetProtocol,
							std::map< std::string, Network::SocketAddress > > YPHostMap;

		// Putters
			/// Insert a new host ( True if the host is really inserted ).
			bool newHost( Network::NetProtocol socket, const std::string& name, const Network::SocketAddress &address );
			/// Delete a host address ( True if the host is really deleted ).
			bool eraseHost( Network::NetProtocol socket, const std::string& name );

		// Getters
			/// Load the host list from file.
			bool loadFromFile(const std::string& fileName);

			/// Get host address trought given socket e name ( logaritmic time implementation ).
			bool getSocketAddress( Network::NetProtocol socket, const std::string& name, Network::SocketAddress &address ) const;

			/// Take the complete host list.
			const std::map< Network::NetProtocol, std::map< std::string, Network::SocketAddress > >& getMap() const;
	};

}} // namespaces

#endif
