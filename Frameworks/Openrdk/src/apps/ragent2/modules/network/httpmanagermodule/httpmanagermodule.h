/**
 * @file
 * @brief This file contains the declaration of HttpManagerModule
 */

#ifndef RDK_MODULE_HTTPMANAGERMODULE
#define RDK_MODULE_HTTPMANAGERMODULE

#include <rdkcore/modules/module.h>
#include <rdkcore/netobjs/tcpserversocket.h>
#include <rdkcore/netobjs/tcpsocket.h>
#include <rdkcore/simplethreads/simplethreads.h>

#include <list>
#include <map>

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of HttpManagerModule here.
 *
 * Please write a description of HttpManagerModule here.
 *
 * @ingroup RAgentModules
 */

class HttpPeerManager;

class HttpManagerModule : public Module {
public:
	HttpManagerModule() { }
	virtual ~HttpManagerModule() { }

	bool initConfigurationProperties();
	bool init();
	void exec();
	void exitRequested();
	void cleanup();
	
private:
	NetObjs::TcpServerSocket serverSocket;
	list<HttpPeerManager*> peerManagers;
};

class HttpPeerManager : public SimpleThreads::Thread {
public:	
	HttpPeerManager(NetObjs::TcpSocket* tcpSocket, Session* session) : connected(true), tcpSocket(tcpSocket), session(session) { }
	~HttpPeerManager();
	
	volatile bool connected;
protected:
	void exec();
	void exitRequested();
private:
	void getHttpParams(string& reqString, map<string, string>& params);

	struct Request {
		string method, res, vers;
		map<string, string> params;
		map<string, string> headers;
		string body;
	};

	void sendHtmlIndex(NetObjs::TcpSocket& tcpSocket);
	void sendPropertyTree(NetObjs::TcpSocket& tcpSocket);
	void sendPropertyValue(NetObjs::TcpSocket& tcpSocket, Request& r);
	
	void sendWebPagesDir(NetObjs::TcpSocket& tcpSocket);
	void sendWebPage(NetObjs::TcpSocket& tcpSocket, Request& r);
	
	void sendErrorResponse(NetObjs::TcpSocket& tcpSocket, const string& header, const string& message);	

	bool readHttpRequest(NetObjs::TcpSocket& tcpSocket, Request& r);
	void processPostRequest(const Request& r);
	
	NetObjs::TcpSocket* tcpSocket;
	Session* session;
};

}} // namespace

#endif
