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

#ifndef RDK2_RNETOBJECTS_RNETMESSAGE
#define RDK2_RNETOBJECTS_RNETMESSAGE

#include <rdkcore/network/nettypes.h>
#include <rdkcore/object/object.h>
#include <string>

#define UDP_MESSAGE_MAXSIZE 3000
#define MAXIMUM_TRANSMIT_UNIT 1500

namespace RDK2 { namespace RNetObjects {


/**
 * Contains a message
 *
 * @author Alberto Ingenito (alberto.ing@gmail.com)
 *
 */

class RNetMessage : public RDK2::Object
{
	public:
		/// List of Type of Message ( Extensions are coordinated here )
		enum Type {
			PEER_IDENTIFICATION = 1,	///< Identification message from the peer
			PROPERTY_UPDATE = 2,		///< A property update
			COORDINATION = 3,		///< Message for coordination tasks
			TEST = 4,			///< Message for testing purpose (feel free to use this :D)
			HARDWARE_SERVER = 5,			///< Message to/from the hardware layer
			PROPERTY_SUBSCRIPTION = 6,	///< A property subscription
			AGENT_CMD = 7,			///< A command sent to tasks' parseCommand()
			HOTSPOT_MESSAGE = 8,		///< HotSpot message (RHotSpot with relative pos, with absolute pos,
							///  or snapshot for the last RHotSpot)
			PROPERTY_TREE = 9,		///< Property tree (with empty objects/values)
			MAPPERDATA = 10,		///< Message for mapper data (laser,rfid,...)
			HBD = 11,			///< Message from Human Body Detection module
			USER = 999,
			USAR_COMSTATION = 12,	///< USARSim ComStation message (to and from)
			HARDWARESERVER = HARDWARE_SERVER	// FIXME DEPRECATED
		};

	public:
		/// Builds an empty RNetMessage
		RNetMessage();
	
		/// Build a complete RNetMessage (the payload is NOT cloned and the netmessage now owns it).
		RNetMessage(	const std::string &addressees,
				const std::string &sender,
				Type type,
				Network::NetProtocol socketType,
				const RDK2::Object* const payload = 0 );

		inline void setRoutedReceivedFrom(std::string recvFrom) { routedReceivedFrom = recvFrom; }
		inline std::string getRoutedReceivedFrom() const { return routedReceivedFrom; }
		inline void setRoutedSendTo(std::string sendTo) { routedSendTo = sendTo; }
		inline std::string getRoutedSendTo() const { return routedSendTo; }
		inline void setUniqueId(unsigned long c) { uniqueId = c; }
		inline unsigned long getUniqueId() const { return uniqueId; }
		virtual ~RNetMessage();
	
	public:
		/// Copy constructor.
		RNetMessage( const RNetMessage& _message );
		
		/// Copy the data inside an RNetMessage.
		RNetMessage& operator=( const RNetMessage& _message );

	// RDK2::Object interface	
	public:
		/// Delete the RDK2::Object contained in the message.
		bool reset();

	public:
		void read(Reader* r) throw (RDK2::ReadingException);
		void write(Writer* w) const throw (RDK2::WritingException);
		RDK2::Object* clone() const;

		bool equals(const Object*p) const;
		std::vector<Object*>  getTestCases() const;

	// Getters
	public:
		static std::size_t getOverheadSize(	const std::string &addressees,
							const std::string &sender,
							Type type,
							Network::NetProtocol socketType );
		static std::size_t getBaseHeadSize();
		/// Return the addressees.
		const std::string& getAddressees() const;
		/// Return the sender string.
		const std::string& getSender() const;
		/// Get type.
		Type getType() const;
		/// Get method.
		Network::NetProtocol getNetProtocol() const;
		/// Get the payload ( NOT CLONED ).
		const RDK2::Object* getPayload() const;

		template<typename T>
		inline const T* getPayloadAs() const {
			return dynamic_cast<const T*>(getPayload());
		}

		double timestamp;
		double getDelay() const;

	private:
		std::string          addressees;         ///< Addressees names.
		std::string          sender;             ///< Sender name.
		std::string          routedReceivedFrom; ///< If routed, this contains the RAgent from which the message has been received
		std::string          routedSendTo;       ///< If routed, this contains the RAgents to which the message has to be sent
		int32_t        uniqueId;           ///< If routed, unique message code (control flooding)
		Type                 type;               ///< Type.
		Network::NetProtocol socketType;         ///< Method.
		const RDK2::Object*  payload;            ///< Pointer to payload.

	static int32_t messageCounter;
};

}} // namespaces

#endif
