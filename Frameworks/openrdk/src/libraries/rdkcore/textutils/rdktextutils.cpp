#include "rdktextutils.h"

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RdkTextUtils"

namespace RDK2 {

bool charIsIn(char c, const string& s)
{
	return s.find(c) != string::npos;
}

ParsedTextConfigLine parseTextConfigLine(const string& line)
{
	ParsedTextConfigLine cl;
	string l = line;
	l = l.substr(l.find_first_not_of(" \t"));	// strips leading whitespaces
	size_t i = 0;

	// read property url
	string propertyUrl;
	for ( ; i < l.size(); i++) {
		if (charIsIn(l[i], "= \t")) break;
		else propertyUrl.push_back(l[i]);
	}

	// skips whitespaces before '='
	while (i < l.size() && l[i++] != '=') ;

	// skips whitespaces after '='
	i = l.find_first_not_of(" \t", i + 1);

	// read if property is a link
	bool isLink = (i < l.size() && l[i] == '@');

	// read property value or property link target
	string value, linkToUrl;
	if (!isLink) {
		bool quotes = false;
		for ( ; i < l.size(); i++) {
			if (!quotes && charIsIn(l[i], "[#")) break;
			if (l[i] == '"') quotes = !quotes;
			else value.push_back(l[i]);	
		}
	}
	else {
		i++;	// skips the '@'
		for ( ; i < l.size(); i++) {
			if (charIsIn(l[i], "[#")) break;
			else linkToUrl.push_back(l[i]);	
		}
	}
	
	return cl;
}

}

