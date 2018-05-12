#include <cstdio>
#include <iostream>
using namespace std;

#include <rdkcore/textutils/rdktextutils.h>
using namespace RDK2;

#define BEGIN_TEST(l, purl, val, linkurl) \
	cl = parseTextConfigLine(l); \
	cout << "Config line: \"" << l << "\":" << endl; \
	cout << "\tProperty URL: \"" << cl.relativeUrl << "\" (\"" << purl << "\") "; \
	if (purl != cl.relativeUrl) { cout << "***** FAILED *****" << endl; ok = false; } \
	else cout << "OK" << endl; \
	cout << "\tProperty value: \"" << cl.value << "\" (\"" << val << "\") "; \
	if (cl.value != val) { cout << "***** FAILED *****" << endl; ok = false; } \
	else cout << "OK" << endl; \
	cout << "\tLink to URL: \"" << cl.linkToUrl << "\" (\"" << linkurl << "\") "; \
	if (linkurl != cl.linkToUrl) { cout << "***** FAILED *****" << endl; ok = false; } \
	else cout << "OK" << endl;

#define TEST_OPTION(opt, val) \
	cout << "\tOption \"" << opt << "\": "; \
	optIt = cl.options.find(opt); \
	if (optIt == cl.options.end()) cout << "NOT FOUND"; \
	else cout << "\"" << optIt->second << "\""; \
	cout << " (\"" << val << "\") "; \
	if (optIt == cl.options.end() || optIt->second != val) { cout << "***** FAILED *****" << endl; ok = false; } \
	else cout << "OK" << endl;

int main(int, char**)
{
	bool ok = true;
	ParsedTextConfigLine cl;
	map<string, string>::iterator optIt;

	BEGIN_TEST("   property1=@/linkedProperty  # this is a comment", "property1", "", "/linkedProperty");
	
	BEGIN_TEST("   /tested/property=this is the value  [OPTION1,OPTION2=VALUE2]  # this is a comment",
		"/tested/property", "this is the value", "");
	TEST_OPTION("OPTION1", "");
	TEST_OPTION("OPTION2", "VALUE2");

	return (ok ? 0 : 1);
}
