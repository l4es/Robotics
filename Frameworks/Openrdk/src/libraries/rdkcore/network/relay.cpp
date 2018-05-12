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

#include <rdkcore/network/udpsocket.h>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <set>

#define VERSION		1.1
#define BUFFERSIZE	0xFFFF

#define _DEBUG
#ifdef _DEBUG
	#define DOUT(a) cout<<setw(4)<<__LINE__<<"|"<<setw(15)<<__func__<<"| "<<a<<endl;
#else
	#define DOUT(a)
#endif

using namespace std;

int main ( int argc, char** argv ) {
	try {
		set<unsigned short> group;
		if ( argc < 3 ) {
			cout<<"relay port forward1 forward2 forward3 ...\n";
			return 0;
		}
		
		// Server port
		int port = -1;
		port = atoi( argv[1] ); 
		if ( !( port > 0x400 && port < 0xFFFF) ) {
			cout<<"All port must be over 1024 and under 65000\n";
			return 0;
		}
		
		// Forward ports
		for ( int i = 2; i < argc; i++ ) {
			int temp = -1;
			temp = atoi( argv[i] );
			if ( !( temp > 0x400 && temp < 0xFFFF) )
				cout<<"warning: All port must be over 1024 and under 65535 ( port "<<temp<<" ignored )\n";
			else if ( temp == port )
				cout<<"warning: Cannot use same port for forward & recive ( port "<<temp<<" ignored )\n";
			else
				group.insert( temp );
		}
		
		// Out a riepilogue
		cout<<"Incoming port = "<<port<<endl;
		for ( set<unsigned short>::const_iterator iter = group.begin(); iter != group.end(); iter++ ) {
			cout<<"Forward port  = "<<*iter<<endl;
		}
		
		Network::UDPSocket udpsocket( port );
		
		string str;
		Network::InetAddress sender;
		Network::InetAddress dest( "127.0.0.1", port );;
		
		DOUT("Waiting");
		
		while ( true ) {
			udpsocket.recv( str, sender );
			
			DOUT("Message received from " << sender.getIPAddress() << ":" << sender.getPort());
			
			for ( set<unsigned short>::const_iterator iter = group.begin(); iter != group.end(); iter++ ) {
				dest.setPort( *iter );
				udpsocket.send( str, dest );
				DOUT("Message sent to "<<dest.getPort());
			}
		}
	
		return 1;
	} catch ( Network::NetException& netexc ) {
		cout<<"An error as occurred: "<<netexc.fullwhat()<<endl;
	}
	return 0;
}

/*
#include <rdkcore/network/udpsocket.h>
#include <iostream>
#include <string.h>
#include <set>

#define VERSION		0.1

using namespace std;
using namespace Network;

static bool help( int argc, const char** argv ) {
	for ( int i = 0; i < argc; i++ ) {
		if (	strcmp ( argv[i], "--help") == 0 ||
			strcmp ( argv[i], "-h" ) == 0 ) {
			// Print use information
			cout<<"UDP Relay - version "<<VERSION<<endl;
			cout<<"This program is part of SPQR-RDK\n\n";
			cout<<"To use:relay port\n\nes1: relay 5000 will open a receiving port on 5000 and a control port on 5001\n";
			cout<<"es2: relay 5000 6000 will open a receiving port on 5000 and a control port on 6000\n";
			cout<<"es3: relay 5000 5001 1500 will open a receiving port on 5000, with a buffer size of 1500 bytes\n\n";
			cout<<"A message 'register' from an udp port to the control port register that port for forwarding\n";
			cout<<"A second message 'bye' from the same udp port unregister that port\n";
		return true;
		}
	}
	return false;
}

class Forwarder {
	public:
		Forwarder( unsigned short forward, unsigned short control, unsigned short buffersize ) throw( NetException ) :
			forward( forward, buffersize ),
			control( control, 8 ) {
			pthread_mutex_init(&registered_mutex, 0);
		}
		
		~Forwarder() {
			pthread_mutex_destroy(&registered_mutex);
		}
		
		void start() throw( NetException ) {
			// Launch forwarding thread
			pthread_create( &receiver_thread, NULL, receiver, this );
			string str;
			InetAddress addr;
			
			// Control command read cicle
			while ( true ) {
				// Get a message
				control.recv( str, addr );
				// If is a registraition message;
				if ( str == "register" ) {
					pthread_mutex_lock(&registered_mutex);
						registered.insert( addr.getPort() );
					pthread_mutex_unlock(&registered_mutex);
					cout<<"port: "<<addr.getPort()<<" registered ( ip: "<<addr.getIPAddress()<<" )\n";
				} else if ( str == "bye" ) {
					pthread_mutex_lock(&registered_mutex);
						registered.erase( addr.getPort() );
					pthread_mutex_unlock(&registered_mutex);
					cout<<"port: "<<addr.getPort()<<" removed ( ip: "<<addr.getIPAddress()<<" )\n";
				}
			}
		}
		
	private:
		UDPSocket forward;
		UDPSocket control;
		set<unsigned short> registered;
		pthread_t receiver_thread;
		pthread_mutex_t registered_mutex;
		
		static void* receiver( void* _this ) { 
			Forwarder* me = (Forwarder*)_this;
			string str;
			InetAddress addr;
			InetAddress dest( "127.0.0.1", 1234 );
			while ( true ) {
				// Receive a packet
				me->forward.recv( str, addr );
				
				pthread_mutex_lock(&me->registered_mutex);
					
					// Send cicle	
				
					for (	set<unsigned short>::const_iterator iter = me->registered.begin();
						iter != me->registered.end();
						iter++ ) {
							dest.setPort( *iter );
							me->forward.send( str, dest );
					}
				
				pthread_mutex_unlock(&me->registered_mutex);
			}
			
			return NULL;
		}
};


int main(int argc, const char** argv) {
	try {
		// Control for help request
		if ( help( argc, argv ) )
			return 1;
	
		// No parameters
		if ( argc < 2 ) {
			cerr<<"Need to specify port at least ( -h --help for help ), exiting\n";
			return 0;
		}
		
		int port = -1;
		int cport = -1;
		int buffer = -1;
		
		// One parameter
		if ( argc == 2 ) {
			port = atoi( argv[1] );
			cport = port + 1;
			buffer = 1500;
		} else if ( argc == 3 ) {
			port = atoi( argv[1] );
			cport = atoi( argv[2] );
			buffer = 1500;
		} else if ( argc == 4 ) {
			port = atoi( argv[1] );
			cport = atoi( argv[2] );
			buffer = atoi( argv[3] );
		}
		
		if ( !(	port > 0 && cport > 0 && buffer > 0 &&
			port < 0xFFFF && port < 0xFFFF && buffer < 0xFFFF &&
			port != cport ) ) {
			cerr<<"Wrong input data\n";
			return 0;
		} else {
			cout<<"Opening relay on:\nport:         "<<port<<"\ncontrol port: "<<cport<<"\nbuffer:       "<<buffer<<endl;
			Forwarder forwarder( port, cport, buffer );
			forwarder.start();
		}
		return 1;

	} catch ( exception& exc ) {
		cout<<exc.what()<<endl;
	}
	return 0;
}*/
