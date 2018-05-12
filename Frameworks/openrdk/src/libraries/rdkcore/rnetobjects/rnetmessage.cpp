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

#include "rnetmessage.h"
#include <rdkcore/object/objectmanager.h>
#include <rdkcore/rprimitive/rstring.h>

#include <sys/time.h>
#include <string>
#include <iostream>

#include <rdkcore/ns.h>
#include <math.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RNetMessage"

//#define _OUT_DEBUG
//#define _IN_DEBUG

#ifdef _OUT_DEBUG
	#define OUT_DEBUG_STREAM(a) { RDK_DEBUG_STREAM(a) }
#else
	#define OUT_DEBUG_STREAM(a)
#endif

#ifdef _IN_DEBUG
	#define IN_DEBUG_STREAM(a) { RDK_DEBUG_STREAM(a) }
#else
	#define IN_DEBUG_STREAM(a)
#endif

namespace RDK2 { namespace RNetObjects {

RDK2_FACTORY(RNetMessage);

using namespace std;

int32_t RNetMessage::messageCounter;

void RNetMessage::read(Reader* r) throw (RDK2::ReadingException)
{
	r->startReading(getClassName());
		addressees = r->readString();
		sender = r->readString();
		type = (Type) r->read_i32();
		socketType = (Network::NetProtocol) r->read_i32();
		uniqueId = r->read_i32();
		routedSendTo = r->readString();
		routedReceivedFrom = r->readString();
		payload = (RDK2::Object*) r->readObject();
	r->doneReading();
}

void RNetMessage::write(Writer* w) const throw (RDK2::WritingException)
{
	if(!payload)
		throw WritingException("Cannot serialize RNetMessage with null payload.");

	w->startWriting(getClassName());
		w->writeString(addressees,"addressees");
		w->writeString(sender,"sender");
		w->write_i32(type,"messageType");
		w->write_i32(socketType,"socketType");
		w->write_i32(uniqueId,"uniqueId");
		w->writeString(routedSendTo,"routedSendTo");
		w->writeString(routedReceivedFrom,"routedReceivedFrom");
		w->writeObject(true, payload, "payload");
	w->doneWriting();
}

std::vector<Object*> RNetMessage::getTestCases() const { 
	RNetMessage * o = new RNetMessage();
	o->addressees = "Barbie BratZ";
	o->sender = "Gi-ai-gio";
	o->type = PEER_IDENTIFICATION;
	o->socketType = Network::TCP_IP;
	o->payload = new RDK2::RPrimitive::RString("Hello baby");

	std::vector<Object*> v;
	v.push_back(o);
	return v;
}

bool  RNetMessage::equals(const Object*p) const {
	const RNetMessage * o = dynamic_cast<const RNetMessage *>(p);
	return o 
		&& (o->addressees == addressees) 
		&& (o->sender     == sender)
		&& (o->type       == type)
		&& (o->socketType == socketType)
		&& (	(!o->payload && !o->payload) 
				|| (o->payload && payload &&payload->equals(o->payload))
		);
}

double RNetMessage::getDelay() const {
	struct timeval tv;
	::gettimeofday( &tv, 0 );
	return tv.tv_sec * 1000. + tv.tv_usec * 0.001 - timestamp;
}

bool RNetMessage::reset()
{
	if(payload) {
		delete payload;
		payload = 0;
	}
	return true;
}

RDK2::Object* RNetMessage::clone() const
{
	return new RNetMessage( addressees, sender, type, socketType, 
		payload ? payload->clone() : 0 );
}

// RNetMessage copy & copyconstructor method -------------------------------------------------
RNetMessage::RNetMessage( const RNetMessage& _message ) :
	RDK2::Object(),
//	Serializable(),
	addressees(_message.addressees),
	sender(_message.sender),
	uniqueId(_message.uniqueId),
	type(_message.type), 
	socketType(_message.socketType) {
	payload =  _message.payload ? _message.payload->clone() : 0;
	
	struct timeval tv;
	::gettimeofday( &tv, 0 );
	timestamp = tv.tv_sec * 1000. + tv.tv_usec * 0.001;
}

RNetMessage& RNetMessage::operator=( const RNetMessage& _message )
{
	reset();
	addressees  = _message.addressees;
	sender     = _message.sender;
	uniqueId  = _message.uniqueId;
	type       = _message.type;
	socketType = _message.socketType;
	payload =  _message.payload ? _message.payload->clone() : 0;
	
	return *this;
}

RNetMessage::RNetMessage() :
	payload( new RString("unset message") ) {
	IN_DEBUG_STREAM("Creating empty RNetMessage");
}

// RNetMessage constructor
RNetMessage::RNetMessage(	const std::string &addressees,
				const std::string &sender,
				Type type,
				Network::NetProtocol socketType, 
				const RDK2::Object* const payload)
:	addressees(addressees), sender(sender), type(type), socketType(socketType), payload(payload)
{
	uniqueId = messageCounter++;
	OUT_DEBUG_STREAM("Creating RNetMessage: to '"<<addressees<<"' from '"<<sender<<"'");
	struct timeval tv;
	::gettimeofday( &tv, 0 );
	timestamp = tv.tv_sec * 1000. + tv.tv_usec * 0.001;
}

// RNetMessage destructor --------------------------------------------------------------------
RNetMessage::~RNetMessage()
{
// 	std::cout<<"deleting: ";
// 	save( std::cout );
// 	std::cout<<endl;
	if ( payload ) delete payload;
}

// RNetMessage working method ----------------------------------------------------------------
const std::string& RNetMessage::getAddressees() const
{
	return addressees;
}

const std::string& RNetMessage::getSender() const
{
	IN_DEBUG_STREAM("Returning Sender: "<<sender);
	return sender;
}

RNetMessage::Type RNetMessage::getType() const
{
	return type;
}

Network::NetProtocol RNetMessage::getNetProtocol() const
{
	return socketType;
}

const RDK2::Object* RNetMessage::getPayload() const
{
	return payload;
}

static size_t charcount( int i ) {
	return (i==0) ? 1 : (size_t)log10f(i) + 1;
}

std::size_t RNetMessage::getOverheadSize(	const std::string &addressees, const std::string &sender,
				Type type, Network::NetProtocol socketType ) {
	static size_t BaseOverHead = getBaseHeadSize();
	return BaseOverHead + addressees.size() + sender.size() + charcount( type ) + charcount( socketType );
}

std::size_t RNetMessage::getBaseHeadSize() {
	std::string empty;
	std::ostringstream oss;
	oss << "<RNetMessage addressees=\"\" ";
	oss << "sender=\"\" ";
	oss << "type=\"\" socketType=\"\">";
	oss <<"</RNetMessage>";
	std::string result = oss.str();
	RDK_DEBUG_STREAM("Base overhead computed now, result: "<<result.size());
	RDK_DEBUG_STREAM("String generated is: " << result );
	
	return oss.str().size();
}

}} // namespaces

