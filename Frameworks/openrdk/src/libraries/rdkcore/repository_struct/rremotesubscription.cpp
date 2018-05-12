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

#include <fstream>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RRemoteSubscription"

#include <rdkcore/object/objectmanager.h>

#include "rremotesubscription.h"

namespace RDK2 { namespace RepositoryNS {

RDK2_FACTORY(RRemoteSubscription);
	
void RRemoteSubscription::read(Reader* r) throw (RDK2::ReadingException)
{
	r->startReading(getClassName());
		completeUrl = r->readString();
		askingHost = r->readString();
		socketType = (Network::NetProtocol) r->read_i32();
		what = (What) r->read_i32();
		when = (When) r->read_i32();
		id = r->read_i32();
		minPeriod = r->read_f32();
		maxPeriod = r->read_f32();
		lastIdReceivedByClient = r->read_i32();
	r->doneReading();
}

void RRemoteSubscription::write(Writer* w) const throw (RDK2::WritingException)
{
	w->startWriting(getClassName());
		w->writeString(completeUrl);
		w->writeString(askingHost);
		w->write_i32(socketType);
		w->write_i32(what);
		w->write_i32(when);
		w->write_i32(id);
		w->write_f32(minPeriod);
		w->write_f32(maxPeriod);
		w->write_i32(lastIdReceivedByClient);
	w->doneWriting();
}

RDK2::Object* RRemoteSubscription::clone() const
{
	RRemoteSubscription* p = new RRemoteSubscription;
	p->completeUrl = this->completeUrl;
	p->askingHost = this->askingHost;
	p->socketType = this->socketType;
	p->what = this->what;
	p->when = this->when;
	p->minPeriod = this->minPeriod;
	p->maxPeriod = this->maxPeriod;
	p->lastIdReceivedByClient = this->lastIdReceivedByClient;
	p->id = this->id;
	return p;
}

string RRemoteSubscription::getStringForVisualization() const {
	ostringstream oss;
	oss << id << ": '" << completeUrl << "' asked by '" << askingHost << "' (";
	oss << Network::netProtocolToString(socketType) << ", ";
	switch (what) {
		case WHAT_UNKNOWN: oss << "UNKNOWN/ANY"; break;
		case STORAGE_VALUE: oss << "STORAGE_VALUE"; break;
		case STORAGE_DIFFS: oss << "STORAGE_DIFFS"; break;
		case QUEUE: oss << "QUEUE"; break;
	}
	oss << ", ";
	switch (when) {
		case WHEN_UNKNOWN: oss << "UNKNOWN/ANY"; break;
		case ON_CHANGE: oss << "ON_CHANGE"; break;
		case PERIODIC: oss << "PERIODIC"; break;
	}
	if (minPeriod != 0.) oss << ", min: " << minPeriod << "ms";
	if (maxPeriod != 0.) oss << ", max: " << maxPeriod << "ms";
	oss << ")";
	return oss.str();
}

}} // namespaces
