#define MODULE_NAME "HttpManagerModule"

#include "httpmanagermodule.h"
#include "httpmanagermodule_names.h"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/ns.h>
#include <rdkcore/filesystem/filesystem.h>
#include <rdkcore/textutils/textutils.h>
#define LOGGING_MODULE MODULE_NAME

#include <cstdio>

namespace RDK2 { namespace RAgent {

bool HttpManagerModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, true);
		session->createInt(PROPERTY_SERVER_PORT, "Server listening port", 8080);
		session->createString(PROPERTY_WEBPAGES_DIR, "Directory where to find web pages", "");
		session->createBool(PROPERTY_WEBPAGES_ENABLED, "Web pages support enabled", false);
		session->createBool(PROPERTY_WEBPAGES_LIST_ALLOWED, "Web pages list allowed", false);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool HttpManagerModule::init()
{
	SESSION_TRY_START(session)
		if (!serverSocket.bind(session->getInt(PROPERTY_SERVER_PORT))) {
			RDK_ERROR_PRINTF("Cannot bind server to port %d (%s)", session->getInt(PROPERTY_SERVER_PORT),
				serverSocket.getLastError().c_str());
			session->end();
			return false;
		}
		RDK_INFO_PRINTF("HTTP Server bound to port %d", session->getInt(PROPERTY_SERVER_PORT));
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void HttpManagerModule::exec()
{
	while (session->dontWait(), !exiting) {
		SESSION_TRY_START(session)
			NetObjs::TcpSocket* tcpSocket = serverSocket.accept();
			if (tcpSocket) {
				RDK_INFO_PRINTF("Connection accepted from %s", tcpSocket->getPeerAddress().toString().c_str());
				
				Session* peerManagerSession =
					session->getRepository()->createSession("Peer manager", "Peer manager", session->getUrlContext());
				HttpPeerManager* peerManager = new HttpPeerManager(tcpSocket, peerManagerSession);
				peerManagers.push_back(peerManager);
				peerManager->start();
				// garbage collection (peer managers whose connection has been closed by the peer)
				for (list<HttpPeerManager*>::iterator it = peerManagers.begin(); it != peerManagers.end(); ) {
					if (!(*it)->connected) {
						(*it)->stop();
						delete *it;
						it = peerManagers.erase(it);
					}
					else ++it;
				}
			}
			else {
				if (!exiting) {
					RDK_ERROR_PRINTF("Error in accepting connections (%s)", serverSocket.getLastError().c_str());
				}
			}
		SESSION_END_CATCH_TERMINATE(session)
	}
}

void HttpManagerModule::exitRequested()
{
	serverSocket.unbind();
}

void HttpManagerModule::cleanup()
{
	RDK_DEBUG_PRINTF("Stopping peer threads");
	for (list<HttpPeerManager*>::iterator it = peerManagers.begin(); it != peerManagers.end(); ++it) {
		(*it)->stop();
		delete *it;
	}
}

void HttpPeerManager::getHttpParams(string& reqString, map<string, string>& params)
{
	if (reqString.find_first_of("?") != string::npos) {
		string sparams = reqString.substr(reqString.find_first_of("?")+1);
		reqString = reqString.substr(0, reqString.find_first_of("?"));
		vector<string> vparams = TextUtils::tokenize(sparams, "&");
		for (size_t i = 0; i < vparams.size(); i++) {
			vector<string> pv = TextUtils::tokenize(vparams[i], "=");
			if (pv.size() == 2) params.insert(make_pair(pv[0], pv[1]));
		}
	}
}

void HttpPeerManager::sendHtmlIndex(NetObjs::TcpSocket& tcpSocket)
{
	SESSION_TRY_START(session)
	string htmlPage = "<html><head><title>OpenRDK agent \"" + session->getRepositoryName() + "\"</title></head>"
		"<body>"
		"<h1>OpenRDK agent \"" + session->getRepositoryName() + "\"</h1>"
		"<p>You retrieved the index page of this OpenRDK agent (a webserver is running as a module inside the agent)."
		" From here you can go to:"
		"<ul>"
		"<li><a href=\"repository/\">The list of the properties in the repository</a></li>";
	if (session->getBool(PROPERTY_WEBPAGES_ENABLED) && session->getBool(PROPERTY_WEBPAGES_LIST_ALLOWED)) {
		htmlPage += "<li><a href=\"webpages/\">The list of the webpages served by this module</a></li>";
	}
	htmlPage += "</ul>"
		"</body></html>";
	tcpSocket << "HTTP/1.1 200 OK\r\n" <<
		"Content-Type: text/html\r\n" <<
		"Content-Length: " << htmlPage.size() << "\r\n\r\n" << htmlPage;
	SESSION_END_CATCH_TERMINATE(session)
}

void HttpPeerManager::sendPropertyTree(NetObjs::TcpSocket& tcpSocket)
{
	string htmlPage = "<html><head><title>OpenRDK agent \"" + session->getRepositoryName() + "\"</title></head>"
		"<h1>OpenRDK agent \"" + session->getRepositoryName() + "\"</h1>"
		"<body><p>This is the list of the properties in this agent's repository:"
		"<ul>";
	SESSION_TRY_START(session)
	vector<Url> names = session->getPropertyNamesStartingWith("/");
	for (size_t i = 0; i < names.size(); i++) {
		htmlPage += "<li><a href=\"" + names[i].substr(1) + "?format=html\">" + names[i] + "</a></li>";
	}
	SESSION_END_CATCH_TERMINATE(session)
	htmlPage += "</ul></p></body></html>";
	tcpSocket << "HTTP/1.1 200 OK\r\n" <<
		"Cache-Control: no-cache\r\n" <<
		"Expires: -1\r\n" <<
		"Content-Type: text/html\r\n" <<
		"Content-Length: " << htmlPage.size() << "\r\n\r\n" << htmlPage;
}

void HttpPeerManager::sendPropertyValue(NetObjs::TcpSocket& tcpSocket, Request& r)
{
	string name = r.res.substr(11);
	RDK2::Object* obj = 0;
	bool sent = false;
	SESSION_TRY_START(session)
		obj = session->getObjectClone(name); // FIXME check what getObjectClone returns if the object is not set (0?)
		if (obj) {
			string objRepresentation;
			if (obj->hasStringForVisualization()) {
				objRepresentation = obj->getStringForVisualization();
				tcpSocket << "HTTP/1.1 200 OK\r\n";
				tcpSocket << "Cache-Control: no-cache\r\n";
				tcpSocket << "Expires: -1\r\n";
				if (r.params["action"] == "edit") {
					tcpSocket << "Content-Type: text/html\r\n";
					objRepresentation = "<html><body><form method=\"POST\" action=\"" + r.res + "?format=html\">"
						"<b>Property " + name + "</b>: "
						"<input value=\"" + obj->getStringRepresentation() + "\" type=\"text\" name=\"" + name + "\"/>"
						"<input type=\"submit\" value=\"Update\"/>"
						"</form>"
						"</body></html>";
				}
				else if (r.params["format"] == "html") {
					tcpSocket << "Content-Type: text/html\r\n";
					objRepresentation = "<html><body><b>Property " + name + "</b>: " + objRepresentation +
						" [<a href=\"" + r.res + "?format=html\">refresh</a>] "
						" [<a href=\"" + r.res + "?action=edit\">edit</a>] " + "<br/><hr/>"
						"<a href=\"/repository/\">Back to repository tree</a></body></html>";
				}
				else tcpSocket << "Content-Type: text/plain\r\n";
				tcpSocket << "Content-Length: " << objRepresentation.size() << "\r\n\r\n";
				tcpSocket << objRepresentation;
				sent = true;
			}
			else {
				if (obj->getClassName() == "RImage") {
					RImage* img = dynamic_cast<RImage*>(obj);
					if (img) {
						if (r.params["format"] == "jpg" || r.params["format"] == "jpeg") {
							char* tfilename = tempnam("/tmp/", "openrdk-");
							if (!img->saveMagick(string(tfilename) + ".jpg")) {
								objRepresentation = "(internal error)";
							}
							else {
								objRepresentation = fsReadFile(string(tfilename) + ".jpg");
								string cmd = string("rm ") + tfilename + ".jpg";
								int r = system(cmd.c_str());
								if (r == -1) {
									RDK_ERROR_PRINTF("Something wrong while deleting a temporary file");
								}
								tcpSocket << "HTTP/1.1 200 OK\r\n";
								tcpSocket << "Cache-Control: no-cache\r\n";
								tcpSocket << "Expires: -1\r\n";
								tcpSocket << "Content-Type: image/jpeg\r\n";
								tcpSocket << "Content-Length: " << objRepresentation.size() << "\r\n\r\n";
								tcpSocket << objRepresentation;
							}
							delete[] tfilename;
							sent = true;
						}
						else if (r.params["format"] == "html") {
							objRepresentation = "<html><head>"
								"<script language=\"JavaScript\">"
								"var count = 0;"
								"function refresh() {"
								"  document.getElementById(\"img\").src = \"" + r.res + "?format=jpeg&\"+count;"
								"  count++;"
								"  setTimeout(\"refresh()\", 1000);"
								"}"
								"setTimeout(\"refresh()\", 1000);"
								"</script></head>"
								"<body>"
								"<img id=\"img\" src=\"" + r.res + "?format=jpeg\"/>"
								"</body>";
								tcpSocket << "HTTP/1.1 200 OK\r\n";
								tcpSocket << "Cache-Control: no-cache\r\n";
								tcpSocket << "Expires: -1\r\n";
								tcpSocket << "Content-Type: text/html\r\n";
								tcpSocket << "Content-Length: " << objRepresentation.size() << "\r\n\r\n";
								tcpSocket << objRepresentation;
						}
					}
				}
				else if (obj->getClassName() == "RMapImage") {
					RMapImage* rmap = dynamic_cast<RMapImage*>(obj);
					if (rmap) {
						RImage* img = rmap->image;
						if (r.params["format"] == "jpg" || r.params["format"] == "jpeg") {
							char* tfilename = tempnam("/tmp/", "openrdk-");
							if (!img->saveMagick(string(tfilename) + ".jpg")) {
								objRepresentation = "(internal error)";
							}
							else {
								objRepresentation = fsReadFile(string(tfilename) + ".jpg");
								string cmd = string("rm ") + tfilename + ".jpg";
								int r = system(cmd.c_str());
								if (r == -1) {
									RDK_ERROR_PRINTF("Something wrong while deleting a temporary file");
								}
								tcpSocket << "HTTP/1.1 200 OK\r\n";
								tcpSocket << "Cache-Control: no-cache\r\n";
								tcpSocket << "Expires: -1\r\n";
								tcpSocket << "Content-Type: image/jpeg\r\n";
								tcpSocket << "Content-Length: " << objRepresentation.size() << "\r\n\r\n";
								tcpSocket << objRepresentation;
							}
							delete[] tfilename;
							sent = true;
						}
						else if (r.params["format"] == "png") {
							char* tfilename = tempnam("/tmp/", "openrdk-");
							if (!img->saveMagick(string(tfilename) + ".png")) {
								objRepresentation = "(internal error)";
							}
							else {
								objRepresentation = fsReadFile(string(tfilename) + ".png");
								string cmd = string("rm ") + tfilename + ".png";
								int r = system(cmd.c_str());
								if (r == -1) {
									RDK_ERROR_PRINTF("Something wrong while deleting a temporary file");
								}
								tcpSocket << "HTTP/1.1 200 OK\r\n";
								tcpSocket << "Cache-Control: no-cache\r\n";
								tcpSocket << "Expires: -1\r\n";
								tcpSocket << "Content-Type: image/png\r\n";
								tcpSocket << "Content-Length: " << objRepresentation.size() << "\r\n\r\n";
								tcpSocket << objRepresentation;
							}
							delete[] tfilename;
							sent = true;
						}
						else if (r.params["format"] == "html") {
							objRepresentation = "<html><head>"
								"<script language=\"JavaScript\">"
								"var count = 0;"
								"function refresh() {"
								"  document.getElementById(\"img\").src = \"" + r.res + "?format=png&\"+count;"
								"  count++;"
								"  setTimeout(\"refresh()\", 1000);"
								"}"
								"setTimeout(\"refresh()\", 1000);"
								"</script></head>"
								"<body>"
								"<img id=\"img\" src=\"" + r.res + "?format=jpeg\"/>"
								"</body>";
								tcpSocket << "HTTP/1.1 200 OK\r\n";
								tcpSocket << "Cache-Control: no-cache\r\n";
								tcpSocket << "Expires: -1\r\n";
								tcpSocket << "Content-Type: text/html\r\n";
								tcpSocket << "Content-Length: " << objRepresentation.size() << "\r\n\r\n";
								tcpSocket << objRepresentation;
						}
					}
				}
			}
		}
	SESSION_END_CATCH_TERMINATE(session)
	if (obj) delete obj;
	if (!sent) {
		sendErrorResponse(tcpSocket, "HTTP/1.1 500 Internal Server Error", "Internal Server Error");
	}
}

void HttpPeerManager::sendWebPage(NetObjs::TcpSocket& tcpSocket, Request& r)
{
	string webpage = r.res.substr(10);
	string d;
	SESSION_TRY_START(session)
		d = session->getString(PROPERTY_WEBPAGES_DIR);
	SESSION_END_CATCH_TERMINATE(session)
	if (fsExists(d + "/" + webpage)) {
		if (webpage.find("..") != string::npos || (webpage.size() > 0 && webpage[0] == '/')) {
			sendErrorResponse(tcpSocket, "HTTP/1.1 403 Forbidden", "Forbidden");
		}
		else {
			string webPage = fsReadFile(d + "/" + webpage);
			tcpSocket << "HTTP/1.1 200 OK\r\n";
			tcpSocket << "Cache-Control: no-cache\r\n";
			tcpSocket << "Expires: -1\r\n";
			tcpSocket << "Content-Type: text/html\r\n";
			tcpSocket << "Content-Length: " << webPage.size() << "\r\n\r\n";
			tcpSocket << webPage;
		}
	}
	else {
		sendErrorResponse(tcpSocket, "HTTP/1.1 404 Not Found", "Webpage not found");
	}
}

void HttpPeerManager::sendWebPagesDir(NetObjs::TcpSocket& tcpSocket)
{
	SESSION_TRY_START(session)
	if (!session->getBool(PROPERTY_WEBPAGES_ENABLED)) {
		sendErrorResponse(tcpSocket, "HTTP/1.1 404 Not Found", "Webpages directory is not active");
	}
	else if (!session->getBool(PROPERTY_WEBPAGES_LIST_ALLOWED)) {
		sendErrorResponse(tcpSocket, "HTTP/1.1 403 Forbidden", "Webpages directory listing forbidden");
	}
	else {
		vector<string> dc;
		if (!fsDirContent(session->getString(PROPERTY_WEBPAGES_DIR), dc)) {
			sendErrorResponse(tcpSocket, "HTTP/1.1 500 Internal Server Error", "Internal Server Error");
		}
		else {
			string htmlPage = "<html><head><title>OpenRDK agent \"" + session->getRepositoryName() + "\"</title></head>"
				"<h1>OpenRDK agent \"" + session->getRepositoryName() + "\"</h1>"
				"<body><p>This is the list of the web pages that can be served by this server:"
				"<ul>";
			for (size_t i = 0; i < dc.size(); i++) {
				if (dc[i] != "." && dc[i] != "..")
					htmlPage += "<li><a href=\"" + dc[i] + "\">" + dc[i] + "</a></li>";
			}
			htmlPage += "</ul></p></body></html>";
			tcpSocket << "HTTP/1.1 200 OK\r\n" <<
				"Cache-Control: no-cache\r\n" <<
				"Expires: -1\r\n" <<
				"Content-Type: text/html\r\n" <<
				"Content-Length: " << htmlPage.size() << "\r\n\r\n" << htmlPage;
		}
	}
	SESSION_END_CATCH_TERMINATE(session)
}

void HttpPeerManager::exec()
{
	Request r;
	while (!exiting) {
		while (readHttpRequest(*tcpSocket, r)) {
			if (!tcpSocket->isConnected()) break;
			if (r.method == "GET" || r.method == "POST") {
				if (r.res == "/" || r.res == "/index.html") {
					sendHtmlIndex(*tcpSocket);
					processPostRequest(r);
				}
				else if (r.res == "/repository") {
					sendErrorResponse(*tcpSocket, "HTTP/1.1 302 Found\r\nLocation: /repository/", "");
				}
				else if (r.res == "/webpages") {
					sendErrorResponse(*tcpSocket, "HTTP/1.1 302 Found\r\nLocation: /webpages/", "");
				}
				else if (r.res.substr(0, 12) == "/repository/") {
					processPostRequest(r);
					if (r.res == "/repository/") sendPropertyTree(*tcpSocket);
					else sendPropertyValue(*tcpSocket, r);
				}
				else if (r.res.substr(0, 10) == "/webpages/") {
					if (r.method == "GET") {
						if (r.res == "/webpages/") sendWebPagesDir(*tcpSocket);
						else sendWebPage(*tcpSocket, r);
					}
					else {
						sendErrorResponse(*tcpSocket, "HTTP/1.1 Method Not Allowed\r\nAllow: GET",
							"HTTP method not allowed, only GET is allowed for webpages");
					}
				}
				else {
					sendErrorResponse(*tcpSocket, "HTTP/1.1 404 Not Found", "Resource not found");
				}
			}
			else {
				sendErrorResponse(*tcpSocket, "HTTP/1.1 405 Method Not Allowed\r\nAllow: GET,POST",
					"HTTP method not allowed, this server only knows GET and POST");
			}
			*tcpSocket << flush;
		}
		if (!tcpSocket->isConnected()) {
			RDK_INFO_PRINTF("Connection closed from %s", tcpSocket->getPeerAddress().toString().c_str());
			connected = false;
			return;
		}
	}
}

bool HttpPeerManager::readHttpRequest(NetObjs::TcpSocket& s, Request& r)
{
	r.headers.clear();
	r.params.clear();
	string l;
	while (getline(s, l)) {
		istringstream iss(l);
		iss >> r.method >> r.res >> r.vers;
		if (r.vers == "HTTP/1.0" || r.vers == "HTTP/1.1") {
			while (getline(*tcpSocket, l)) {
				if (l.size() && l[l.size()-1] == '\r') l = l.substr(0, l.size()-1);
				if (l == "") break;
				size_t a = l.find_first_of(":");
				if (a != string::npos) {
					r.headers.insert(make_pair(TextUtils::trim(l.substr(0, a)),
						TextUtils::trim(l.substr(a+1))));
				}
				else {
					RDK_ERROR_PRINTF("Malformed HTTP header line: '%s'", l.c_str());
				}
			}
			getHttpParams(r.res, r.params);
			if (l == "") break;
		}
		else if (r.vers != "") {	// if no protocol version is given, it means pre-HTTP/1.0
			RDK_ERROR_PRINTF("HTTP version '%s' is not supported", r.vers.c_str());
		}
	}
	if (r.headers["Content-Length"] != "") {
		size_t bodylength = atoi(r.headers["Content-Length"].c_str());
		char* buffer = new char[bodylength + 1];
		s.read(buffer, bodylength);
		buffer[bodylength] = '\0';
		r.body = string(buffer);
		delete[] buffer;
	}
	else r.body = "";
	return s;
}

string urldecode(const string& s)
{
	if (s.size() < 3) return s;
	string r;
	for (size_t i = 0; i < s.size(); i++) {
		if (s[i] == '%' && i < s.size() - 2) {
			char c = 0;
			c += (isdigit(s[i+1]) ? s[i+1] - '0' : ((tolower(s[i+1]) - 'a')) + 10) * 16;
			c += (isdigit(s[i+2]) ? s[i+2] - '0' : ((tolower(s[i+2]) - 'a')) + 10) * 1;
			i+=2;
			r += c;
		}
		else if (s[i] == ' ') r += " ";
		else r += s[i];
	}
	return r;
}

void decodebody(const string& s, map<string, string>& pairs)
{
	vector<string> vparams = TextUtils::tokenize(s, "&");
	for (size_t i = 0; i < vparams.size(); i++) {
		vector<string> pv = TextUtils::tokenize(vparams[i], "=");
		if (pv.size() == 2) pairs.insert(make_pair(pv[0], pv[1]));
	}
}

void HttpPeerManager::processPostRequest(const Request& r)
{
/*	printf("Headers:\n");
	for (map<string, string>::const_iterator it = r.headers.begin(); it != r.headers.end(); ++it) {
		printf("'%s' = '%s'\n", it->first.c_str(), it->second.c_str());
	}
	printf("Request parameters:\n");
	for (map<string, string>::const_iterator it = r.params.begin(); it != r.params.end(); ++it) {
		printf("'%s' = '%s'\n", it->first.c_str(), it->second.c_str());
	}
	printf("Body:\n");*/
	map<string, string> m;
	decodebody(r.body, m);
	for (map<string, string>::iterator it = m.begin(); it != m.end(); ++it) {
		SESSION_TRY_START(session)
		session->getRepository()->setPropertyValueFromTextConfig(urldecode(it->first), urldecode(it->second), "/", session);
		session->valueChanged(urldecode(it->first));
		SESSION_END_CATCH_TERMINATE(session)
	}
// 	printf("\n");
}

void HttpPeerManager::sendErrorResponse(NetObjs::TcpSocket& s, const string& header, const string& message)
{
	s << header << "\r\n" << "Content-Length: " << message.size() << "\r\n\r\n" << message;
}

void HttpPeerManager::exitRequested()
{
	tcpSocket->disconnect();
}

HttpPeerManager::~HttpPeerManager()
{
	delete tcpSocket;
}

MODULE_FACTORY(HttpManagerModule);

}} // namespace
