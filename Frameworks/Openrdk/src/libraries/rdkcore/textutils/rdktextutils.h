#ifndef RDK_RDKCORE_RDKTEXTUTILS
#define RDK_RDKCORE_RDKTEXTUTILS

#include <rdkcore/repository_struct/url.h>
#include <map>
#include <string>

namespace RDK2 {

using namespace RepositoryNS;
using namespace std;

struct ParsedTextConfigLine {
	ParsedTextConfigLine() : propertyUrl(""), linkToUrl("") { }
	Url propertyUrl;		//> property url (relative, as is in the config file)
	string value;			//> string representation of the value; it is "" if it is a link
	Url linkToUrl;			//> url that this property points to, without the @; it is "" if it is not a link
	map<string, string> options;	//> option pairs, e.g., TRANSPORT=UDP
};

ParsedTextConfigLine parseTextConfigLine(const string& line);

}

#endif

