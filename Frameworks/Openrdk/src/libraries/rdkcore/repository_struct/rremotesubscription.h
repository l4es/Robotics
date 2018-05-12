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

#ifndef RDK2_RFOREIGNPROPERTIES_RREMOTESUBSCRIPTION
#define RDK2_RFOREIGNPROPERTIES_RREMOTESUBSCRIPTION

#include <sys/time.h>

#include <rdkcore/object/object.h>
#include <rdkcore/rnetobjects/rnetmessage.h>
#include <rdkcore/time/time.h>

#include "url.h"

namespace RDK2 { namespace RepositoryNS {

using RDK2::RNetObjects::RNetMessage;
using RDK2::Time::Timestamp;

struct RRemoteSubscription: public RDK2::Object {

	enum What {
		WHAT_UNKNOWN = 0,
		WHAT_ANY = WHAT_UNKNOWN,
		STORAGE_VALUE = 1,
		STORAGE_DIFFS = 2,
		QUEUE = 3
	} what;
	
	enum When {
		WHEN_UNKNOWN = 0,
		WHEN_ANY = WHEN_UNKNOWN,
		ON_CHANGE = 1,
		PERIODIC = 2
	} when;

	/// url of the property, used when the property subscription is sent
	Url completeUrl;
	string askingHost;
	Network::NetProtocol socketType;

	/// (now it is in milliseconds, FIXME -> seconds)
	/// if when == PERIODIC, the update will be send when maxPeriod has passed from last sending
	/// if when == ON_CHANGE, the update will be send when the property has changed and minPeriod has passed
	///    or maxPeriod has passed (even if the property has not changed)
	double minPeriod;
	double maxPeriod;
	
	/// FIXME used so far only for RImages
	unsigned int lastIdReceivedByClient;
	
	uint id;
	
	/// used only if socketType == UDP_IP (UDP subscription are refreshed)
	struct timeval timestamp;
	
	/// timestamp of the last sending (only if periodic)
	Timestamp lastSendingTimestamp;

	RRemoteSubscription() :
		what(WHAT_UNKNOWN),
		when(WHEN_UNKNOWN),
		completeUrl(""),
		askingHost(""),
		socketType(Network::NETPROTOCOL_UNKNOWN),
		minPeriod(0.),
		maxPeriod(0.),
		lastIdReceivedByClient(0),
		id(0)
	{ }

	RDK2::Object* clone() const;
	void write(Writer* w) const throw (WritingException);
	void read(Reader* r) throw (ReadingException);

	bool hasStringForVisualization() const { return true; }
	string getStringForVisualization() const;
};

}} // namespaces

#endif
