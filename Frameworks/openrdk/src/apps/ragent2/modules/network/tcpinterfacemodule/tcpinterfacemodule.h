/**
 * @file 
 * @ingroup RAgentModules
 *
 * This file contains the class TcpInterfaceModule
 *
 */

#ifndef RDK_MODULE_TCPINTERFACEMODULE
#define RDK_MODULE_TCPINTERFACEMODULE

#include <rdkcore/modules/module.h>

#include <rdkcore/simplethreads/simplethreads.h>
#include <rdkcore/network/tcpserversocket.h>
#include <rdkcore/network/tcpsocket.h>


#include <list>
#include <map>



namespace RDK2 { namespace RAgent {

class DhsThread;

/**
 * @brief An interface for external programs via TCP.
 *
 * Other programs can connect to this interface and, using a
 * text-based protocol, they can access the repository.
 *
 * An example client is RobotClient.
 */
class TcpInterfaceModule : public Module {
public:
	TcpInterfaceModule() : tcpServerSocket(0), tcpSocket(0) { }

	virtual ~TcpInterfaceModule() { }

	bool initConfigurationProperties();
	bool init();
	void exec();
	void exitRequested();
	void cleanup();
	

protected:
	/**
	 * @brief Process received command
	 */
	Network::TCPServerSocket* tcpServerSocket;
	Network::TCPSocket* tcpSocket;
	list<DhsThread*> peerManagers;

	string currentDir;
};

class DhsThread : public SimpleThreads::Thread {
public:	
	DhsThread(Network::TCPSocket* tcpSocket, Session* session);
	~DhsThread();
	
	volatile bool connected;
	Network::TCPSocket* tcpSocket;
	
protected:
	void exec();
	void exitRequested();
	void processCmd(cstr cmd);
private:
		
	Session* session;
};

}}
 // namespace

#endif
