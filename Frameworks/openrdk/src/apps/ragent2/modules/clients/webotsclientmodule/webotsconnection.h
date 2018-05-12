#ifndef __WEBOTSCONNECTION_H__
#define __WEBOTSCONNECTION_H__

#include "webots.h"
#include <nao/object/rnaojoints.h>
#include <string>

class WebotsConnection {
public:
	WebotsConnection();
	~WebotsConnection();

	bool connect(int playnum, const std::string host, int port);
	void close();

	bool sendCmd(RDK2::RNaoJoints &joints);
	bool sendTeleport(WebotsSupervisorData &wsd);
	bool sendCmd(unsigned char * buffer, size_t size);

	bool receive(WebotsData &wdata);
	bool receive(WebotsSupervisorData &wsd);
	bool receive(unsigned char * buffer, size_t size);

	static const int SUPERVISOR = 0;

private:
	int commsockfd, clientsockfd, port;
};

#endif
