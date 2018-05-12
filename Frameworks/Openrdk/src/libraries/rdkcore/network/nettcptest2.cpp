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

#include <rdkcore/network/tcpserversocket.h>
#include <rdkcore/network/tcpsocket.h>
#include <iostream>
#include <signal.h>

using namespace std;
using namespace Network;

void* sender( TCPSocket* );
void* receiver( TCPSocket* );

TCPSocket* tcpstream = NULL;

int main(int argc, const char** argv) {
	pthread_t receiver_thread;
	if ( argc < 2 ) {
		cout<<"server: nettcptest2 port\n";
		return 0;
	}
	
	try {
		while ( true ) {
			cout<<"Listening " << atoi(argv[1]) << endl;
			TCPServerSocket server( atoi(argv[1]) );
			tcpstream = server.accept();
			cout<<"Incoming connection" << endl;
			pthread_create( &receiver_thread, NULL, (void*(*)(void*))receiver, tcpstream );
			sender(tcpstream);
			pthread_join( receiver_thread, NULL);
			delete tcpstream;
			cout<<"Connection reset\n";
		}
	} catch ( Network::NetException& netexc ) {
		cerr<<"An error as occurred: " << netexc.fullwhat() << endl;
	}
	
	return 0;
}

void* sender(TCPSocket* tcpstream) {
	try {
		int character;
		while ( ( ( character = cin.get() ) != EOF ) && tcpstream->put( (char)character) && tcpstream->good() )
			if ( character == '\n' )
				tcpstream->flush();
	} catch ( Network::NetException& netexc ) {
		cerr<<"An error as occurred: " << netexc.fullwhat() << endl;
	}
	return NULL;
}

void* receiver(TCPSocket* tcpstream) {
	try {
		string stringa;
		int character;
		while ( ( ( character = tcpstream->get() ) != EOF ) && cout.put( (char)character) && tcpstream->good() )
			if ( character == '\n' )
				cout.flush();
		cout<<"Connection reset\n";
	} catch ( Network::NetException& netexc ) {
		cerr<<"An error as occurred: " << netexc.fullwhat() << endl;
	}
	return NULL;
}
