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

#include "repository.h"

#include "session.h"

namespace RDK2 { namespace RepositoryNS {

Url Session::getAbsoluteUrl(CUrl url) { return url.contextualize(urlContext); }
string Session::getRepositoryName() const { return repository->getRepositoryName(); }
string Session::getSessionName() const { return sessionName; }
string Session::getDescription() const { return description; }

void Session::checkStarted() throw (InvalidOperation) {
	if (!started) throw InvalidOperation("Session '" + sessionName + "' was not started");
}

bool Session::haveSameObject(cstr url1, cstr url2) throw (NoSuchProperty, InvalidOperation)
{
	Property* p1 = lockAndGetProperty(url1);
	Property* p2 = lockAndGetProperty(url2);
	RDK2::Object* obj1 = p1->getObjectL();
	RDK2::Object* obj2 = p2->getObjectL();
	bool r = (obj1 && obj2 && obj1 == obj2);
	unlockProperty(p2);
	unlockProperty(p1);
	return r;
}

Property* Session::lockAndGetProperty(CUrl url, bool followLinks)
	throw (InvalidOperation, NoSuchProperty)
{
	checkStarted();
	Url aurl = getAbsoluteUrl(url);
	if (!aurl.isAbsolute() && !aurl.isComplete())
		throw InvalidOperation("Url has to be absolute or complete ('" + aurl + "')");

	Property* p = 0;
	for (int redirCount = 0; redirCount < (followLinks ? MAX_LINK_LEVELS : 1); redirCount++) {
		try {
			// throws NoSuchProperty (also if it has been deleted)
			p = repository->getProperty(aurl, this);
		}
		catch (const NoSuchProperty& e) {
			throw(e);
		}
		catch (const SessionException& e) {
			throw(e);
		}
		Profiler::lock((sessionName + ":" + aurl).c_str());

		p->lock(HERE);
		if (p->isLink()) {
			aurl = p->linkTo;
			p->unlock();
			Profiler::unlock((sessionName + ":" + aurl + "(followLinks)").c_str());
		}
		else {
			return p;
		}
	}
	if (followLinks) throw InvalidOperation("Too many link levels");
	else {
		if (p) return p;
		else throw InvalidOperation("Something weird has happened! ");
	}
}

void Session::unlockProperty(Property* p)
{
	p->unlock();
	Profiler::unlock((sessionName + ":" + p->url).c_str());
}

}} // namespaces
