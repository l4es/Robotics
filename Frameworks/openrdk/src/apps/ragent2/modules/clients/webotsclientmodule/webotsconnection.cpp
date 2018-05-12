#include "webotsconnection.h"

#include <iostream>
#include <sys/types.h>
#include <math.h>
#include <cstdlib>
#include <cstdio>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>


WebotsConnection::WebotsConnection() {
	commsockfd   = 0;
	clientsockfd = 0;
//	log = std::fstream("test.txt", std::fstream::out);
}

WebotsConnection::~WebotsConnection() {}

void WebotsConnection::close() {
	if (commsockfd)   ::close(commsockfd);
	if (clientsockfd) ::close(clientsockfd);
//	::log.close();
}

bool WebotsConnection::connect(int playnum, const std::string host, int port) {
	struct sockaddr_in serv_addr;
	memset ( &serv_addr, 0, sizeof(serv_addr) );
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(host.c_str());

	bool connected = false;
	clientsockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (clientsockfd < 0) return false;
	serv_addr.sin_port = htons(port+playnum);
	int x = 0;
	while (!connected && x < 3) {
		connected = true;
		std::cout << "Connecting to " << host << " : " << port+playnum << std::endl;
		fflush(stdout);
		if (::connect(clientsockfd, (struct sockaddr*) &serv_addr, sizeof(serv_addr))<0) {
			connected = false;
			usleep(1000000);
		}
		x++;
	}
	if (!connected) return false;

	if (playnum == SUPERVISOR) return true;

	commsockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (commsockfd < 0) return false;
	serv_addr.sin_port = htons(port+playnum+100);
	x = 0;
	connected = false;
	while (!connected && x < 3) {
		connected = true;
		std::cout << "Connecting to " << host << " : " << port+playnum+100 << std::endl;
		if (::connect(commsockfd, (struct sockaddr*) &serv_addr, sizeof(serv_addr))<0) {
			connected = false;
			usleep(1000000);
		}
		x++;
	}
	return connected;
}

bool WebotsConnection::sendCmd(RDK2::RNaoJoints &joints) {
	float jointsArray[NAO_JOINTS_NUMBER];
	for (int i = 0; i < NAO_JOINTS_NUMBER; i++)
	{
		jointsArray[i] = joints[i];
	}
	return sendCmd((unsigned char *) &jointsArray,sizeof(jointsArray));
}

bool WebotsConnection::sendTeleport(WebotsSupervisorData &wsd) {
	return sendCmd((unsigned char *) &wsd,sizeof(wsd));
}

bool WebotsConnection::sendCmd(unsigned char * pointer, size_t size) {
	int ret = 0;
	size_t sent = 0;

	while (sent < size) {
		ret = write(commsockfd, pointer, size - sent);
		if (ret > 0) {
			sent += ret;
			pointer += ret;
		}
	}
	return true;
}

bool WebotsConnection::receive(WebotsData &wdata) {
	return receive((unsigned char *) &wdata, sizeof(wdata));
}

bool WebotsConnection::receive(WebotsSupervisorData &wsd) {
	return receive((unsigned char *) &wsd, sizeof(wsd));
}

bool WebotsConnection::receive(unsigned char * pointer, size_t size) {
	int ret = 0;
	size_t received = 0;
	while (received < size) {
		ret = read(clientsockfd, pointer, size - received);
		if (ret >= 0) {
			received += ret;
			pointer += ret;
		}
		else return false;
	}
	return true;
}
