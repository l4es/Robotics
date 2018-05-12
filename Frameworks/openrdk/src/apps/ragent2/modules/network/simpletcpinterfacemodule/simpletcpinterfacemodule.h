/**
 * @file 
 * @ingroup RAgentModules
 *
 * This file contains the class SimpleTcpInterfaceModule
 *
 */

#ifndef RDK2_MODULE_SIMPLETCPINTERFACEMODULE
#define RDK2_MODULE_SIMPLETCPINTERFACEMODULE

#include <rdkcore/modules/module.h>
#include <rdkcore/network/tcpserversocket.h>
#include <rdkcore/network/tcpsocket.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief An interface for external programs via TCP.
 *
 * Other programs can connect to this interface and, using a
 * text-based protocol, they can access the repository.
 *
 * An example client is RobotClient.
 */
class SimpleTcpInterfaceModule : public Module {
public:
	SimpleTcpInterfaceModule() : tcpServerSocket(0), tcpSocket(0) { }
	virtual ~SimpleTcpInterfaceModule() { }

	bool initConfigurationProperties();
	bool init();
	void exec();
	void exitRequested();
	void cleanup();

protected:
	/**
	 * @brief Process received command
	 */
	void processCmd(cstr cmd, TCPSocket* tcpSocket);
	
	TCPServerSocket* tcpServerSocket;
	TCPSocket* tcpSocket;

	string currentDir;
};

}} // namespace

#endif
