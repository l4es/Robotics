#include <rdkcore/repository_struct/url.h>
using namespace RDK2;

#include <cstdio>
#include <iostream>
using namespace std;

int main(int, char**)
{
	Url u("");
	bool ok = true;

#define TEST_IT(u, s, e) cout << #s ": expected '" << e << "', is '" << u.s << "'"; \
	if (u.s != e) { cout << " ***** FAILED *****" << endl; ok = false; } else { cout << " OK" << endl; }

	u = "rdk://host1/module2/property3";
	cout << endl << "URL '" << u << "'" << endl;
	TEST_IT(u, getProtocol(), "rdk");
	TEST_IT(u, getHost(), "host1");
	TEST_IT(u, getPath(), "/module2/property3");
	TEST_IT(u, isComplete(), true);
	TEST_IT(u, isAbsolute(), false);
	TEST_IT(u, isRelative(), false);
	TEST_IT(u, contextualize("rdk://anotherHost"), "rdk://host1/module2/property3");
	TEST_IT(u, contextualize(""), "rdk://host1/module2/property3");
	TEST_IT(u, decontextualize("rdk://anotherHost"), "rdk://host1/module2/property3");
	TEST_IT(u, decontextualize("rdk://host1"), "/module2/property3");

	u = "/module2/property3";
	cout << endl << "URL '" << u << "'" << endl;
	TEST_IT(u, getProtocol(), "");
	TEST_IT(u, getHost(), "");
	TEST_IT(u, getPath(), "/module2/property3");
	TEST_IT(u, isComplete(), false);
	TEST_IT(u, isAbsolute(), true);
	TEST_IT(u, isRelative(), false);
	TEST_IT(u, contextualize("rdk://anotherHost"), "rdk://anotherHost/module2/property3");
	TEST_IT(u, contextualize("rdk://anotherHost/prefix"), "rdk://anotherHost/prefix/module2/property3");
	TEST_IT(u, contextualize(""), "/module2/property3");
	TEST_IT(u, decontextualize("rdk://anotherHost"), "/module2/property3");
	TEST_IT(u, decontextualize("rdk://host1"), "/module2/property3");

	u = "protocol://host1/module2/property3";
	cout << endl << "URL '" << u << "'" << endl;
	TEST_IT(u, getProtocol(), "protocol");
	TEST_IT(u, getHost(), "host1");
	TEST_IT(u, getPath(), "/module2/property3");
	TEST_IT(u, isComplete(), true);
	TEST_IT(u, isAbsolute(), false);
	TEST_IT(u, isRelative(), false);
	TEST_IT(u, contextualize("rdk://anotherHost"), "protocol://host1/module2/property3");
	TEST_IT(u, contextualize(""), "protocol://host1/module2/property3");
	TEST_IT(u, decontextualize("rdk://anotherHost"), "protocol://host1/module2/property3");
	TEST_IT(u, decontextualize("protocol://host1"), "/module2/property3");

	u = "file:this/is/a/file";
	cout << endl << "URL '" << u << "'" << endl;
	TEST_IT(u, getProtocol(), "file");
	TEST_IT(u, getHost(), "");
	TEST_IT(u, getPath(), "this/is/a/file");
	TEST_IT(u, isComplete(), true);

	u = "file:/this/is/a/file";
	cout << endl << "URL '" << u << "'" << endl;
	TEST_IT(u, getProtocol(), "file");
	TEST_IT(u, getHost(), "");
	TEST_IT(u, getPath(), "/this/is/a/file");
	TEST_IT(u, isComplete(), true);

	return (ok ? 0 : 1);
}
