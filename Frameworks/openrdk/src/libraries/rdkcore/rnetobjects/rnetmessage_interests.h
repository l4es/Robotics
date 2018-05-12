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

#ifndef RDK2_RNETOBJECTS_RNETMESSAGE_INTEREST
#define RDK2_RNETOBJECTS_RNETMESSAGE_INTEREST

#include <rdkcore/posixqueues/interests.h>
#include "rnetmessage.h"

namespace RDK2 { namespace RNetObjects {

/** Interesse verso messaggi di un certo tipo */
struct MessageWithTypeQueueFilter : public PosixQueues::MyInterests<RDK2::Object> {
	
	MessageWithTypeQueueFilter(int type): type(type) {}
	
	bool areYouInterested( const RDK2::Object* o ) {
		const RNetMessage* m = dynamic_cast<const RNetMessage*>(o);
		return m && type == m->getType();
	}
	private:
		int type;
};

}} // namespaces

#endif

