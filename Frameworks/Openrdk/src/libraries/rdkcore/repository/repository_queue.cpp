/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
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

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "Repository"

#include "repository.h"

namespace RDK2 { namespace RepositoryNS {

// FIXME the only module that uses this is remotepropertymanager
bool Repository::queuePush(CUrl url, RDK2::Object* object) throw (NoSuchProperty, InvalidOperation)
{
	Property* p = getProperty(url);
	bool res = false;
	p->lock(HERE);
	try {
		if (p->getObjectClassName() == "") p->setObjectClassName(QUEUE_CLASSNAME);	// XXX are we sure?
		if (!p->isQueue()) throw InvalidOperation("Property '" + url + "' is not a queue");
		RObjectPipeQueue* queue = dynamic_cast<RObjectPipeQueue*>(getProperty(url)->object);
		//RDK_ERROR_PRINTF("Pushing something in '%s' (%p)", url.c_str(), queue);
		if (p->url.isComplete() && p->url.getHost() != getRepositoryName()) {
			RDK_DEBUG_PRINTF("Sending object to a queue in repository '%s'",
				p->url./*getHost().*/c_str());
			sendObjectsToRemoteQueueSubscribers(p->url, object);
		}
		if (queue) {
			res = queue->push(object, HERE);
			queue->gc(HERE);
		}
		// NOTE: sessions listening (which have subscribed this queue) are awaken by the queue itself
	}
	catch (...) { } // FIXME
	p->unlock();
	return res;
}


}} // namespaces
