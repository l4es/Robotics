#ifndef REPOSITORY_APPLICATION_PROTOCOL_HANDLER_H
#define REPOSITORY_APPLICATION_PROTOCOL_HANDLER_H

#include <rdkcore/repository_struct/url.h>
#include <rdkcore/repository/session.h>

#include <string>

namespace RDK2 {

using namespace RDK2::RepositoryNS;
using namespace std;

class ApplicationProtocolHandler {
public:
	inline void setRepository(Repository* repository) { this->repository = repository; }
	inline Repository* getRepository() { return this->repository; }

	virtual void remotePropertyRequested(Session* session, const Url& url) = 0;

	virtual void setRemotePropertyOption(Session* session, const Url& url, const string& option, const string& value = "") = 0;

	virtual ~ApplicationProtocolHandler() {}

protected:
	Repository* repository;
};

}

#endif

