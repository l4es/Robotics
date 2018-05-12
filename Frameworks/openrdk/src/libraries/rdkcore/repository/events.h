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

#ifndef RDK2_REPOSITORY_EVENT
#define RDK2_REPOSITORY_EVENT

#include <string>
#include <rdkcore/time/time.h>

namespace RDK2 { namespace RepositoryNS {

using namespace RDK2::Time;

#define EVENT_DEFAULT_CLONE(c) virtual c* clone() { return new c(*this); }
#define EVENT_DEFAULT_DESTRUCTOR(c) virtual ~c() { }
#define EVENT_DEFAULT_FUNCTIONS(c) EVENT_DEFAULT_CLONE(c) EVENT_DEFAULT_DESTRUCTOR(c)

#define DECLARE_EVENT_HANDLER(x) inline static bool x##Static(RDK2::RepositoryNS::Session::SessionEventObserver* observer, const RDK2::RepositoryNS::Event* e) \
	{ return ((EVENT_HANDLER_CLASS*)observer)->x(e); }
#define SESSION_EVENT(x) RDK2::RepositoryNS::Session::SessionEventHandler(this, &EVENT_HANDLER_CLASS::x##Static)

struct Event {
	Event(string type) : type(type) { }
	EVENT_DEFAULT_FUNCTIONS(Event);
	inline bool instanceOf(const string& t) const { return type.substr(0, t.size()) == t; }
	Timestamp timestamp;
	string type;
};

struct EventProperty : public Event {
	EventProperty(Url url) : Event("EventProperty"), propertyUrl(url) { }
	EVENT_DEFAULT_FUNCTIONS(EventProperty);
	Url propertyUrl;
};

#define DECLARE_EVENT_PROPERTY(event, parent) struct event : public parent { \
	event(Url url) : parent(url) { type = #event; } EVENT_DEFAULT_FUNCTIONS(event); };

DECLARE_EVENT_PROPERTY(EventPropertyUpdate, EventProperty);
DECLARE_EVENT_PROPERTY(EventPropertyUpdateDiff, EventPropertyUpdate);
DECLARE_EVENT_PROPERTY(EventPropertyUpdateValue, EventPropertyUpdate);
DECLARE_EVENT_PROPERTY(EventPropertyTree, EventProperty);
DECLARE_EVENT_PROPERTY(EventPropertyTreeAdded, EventPropertyTree);
DECLARE_EVENT_PROPERTY(EventPropertyTreeDeleted, EventPropertyTree);
DECLARE_EVENT_PROPERTY(EventPropertyQueuePush, EventProperty);

struct EventTimer: public Event {
	EventTimer() : Event("EventTimer") { }
};

}} // namespace

#endif
