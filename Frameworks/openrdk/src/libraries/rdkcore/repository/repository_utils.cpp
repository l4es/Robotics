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

#include <errno.h>

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "Repository"

#include "repository.h"

//#define DEBUG_THIS
#ifdef DEBUG_THIS
#define DOUT(a, args...) RDK_DEBUG_PRINTF(a, ## args)
#else
#define DOUT(a, args...)
#endif

namespace RDK2 { namespace RepositoryNS {

Property* Repository::getProperty(CUrl url, Session* session) throw (NoSuchProperty)
{
	if (isLocalProperty(url)) return getLocalProperty(url);
	else return getRemoteProperty(url, session);
}

Property* Repository::getLocalProperty(CUrl aurl) throw (NoSuchProperty)
{
	Url url = aurl;
	if (url.isComplete()) {
		if (url.getHost() != getRepositoryName()) {
			throw NoSuchProperty("Property '" + url + "' is not mine!");
		}
		else {
			url = url.getPath();
		}
	}
	propertiesMutex.lock(HERE);
	map<Url, Property*>::iterator it = localProperties.find(url);
	if (it == localProperties.end()) {
		propertiesMutex.unlock();
		throw NoSuchProperty("Property '" + url + "' does not exists");
	}
	propertiesMutex.unlock();
	return it->second;
}

// FIXME deprecated
Property* Repository::getProperty(CUrl aurl) throw (NoSuchProperty, InvalidOperation)
{
	if (!aurl.isAbsolute() && !aurl.isComplete()) 
		throw InvalidOperation("Url has to be absolute or complete ('" + aurl + "')");
	Url url = aurl;
	Property* p;
	propertiesMutex.lock(HERE);
	for (int redirCount = 0; redirCount < MAX_LINK_LEVELS; redirCount++) {
		if (isLocalProperty(url)) {
			if (url.isComplete()) url = url.getPath();
			map<Url, Property*>::iterator it = localProperties.find(url);
			if (it == localProperties.end()) {
				propertiesMutex.unlock();
				throw NoSuchProperty("Property '" + (string) url + "' does not exists");
			}
			p = it->second;
			if (p->deleted) {
				propertiesMutex.unlock();
				throw NoSuchProperty("Property '" + (string) url + "' has been deleted");
			}
			if (p->isLink()) url = p->linkTo;
			else {
				propertiesMutex.unlock();
				return p;
			}
		}
		else {
			propertiesMutex.unlock();
			return getRemoteProperty(url, 0);
			//throw InvalidOperation("Old Repository::getProperty used for property '" + url + "'");
		}
	}
	propertiesMutex.unlock();
	throw InvalidOperation("Too many link levels");
}

bool Repository::isLocalProperty(CUrl url) throw()
{
	return (url.isAbsolute() || (url.isComplete() && url.getHost() == repositoryName));
}

Url Repository::normalizeUrl(CUrl url) throw()
{
	// case url absolute: return url
	// case url relative: return url
	// case url complete: return url without host if host is this host, else return url
	if (!url.isComplete()) return url;
	else {
		if (url.getHost() == getRepositoryName()) return url.getPath();	
		else return url;
	}
}

}} // namespaces
