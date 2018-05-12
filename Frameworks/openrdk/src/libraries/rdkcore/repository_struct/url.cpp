/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <string>
#include <sys/types.h>
#include <vector>

#include <rdkcore/textutils/textutils.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "Url"

#include "url.h"

// #define DEBUG_THIS

#ifdef DEBUG_THIS
#warning a lot of debug
#define DOUT(a, args...) RDK_DEBUG_PRINTF(a, ## args)
#else
#define DOUT(a, args...)
#endif

namespace RDK2 { namespace RepositoryNS {

using namespace std;
using namespace RDK2::TextUtils;

Url::Url(const string& urlString) throw (MalformedUrlException)
: string(normalize(urlString))
{
	checkUrlString(urlString);
}

Url::Url(const char* urlString) throw (MalformedUrlException)
: string(normalize(string(urlString)))
{
	checkUrlString(string(urlString));
}

Url& Url::operator=(const Url& url) throw (MalformedUrlException)
{
	checkUrlString(url);
	string::operator=(normalize((const string&) url));
	return *this;
}

Url Url::contextualize(const Url& context) const throw (MalformedUrlException)
{
	DOUT("Url to contextualize: '%s'; context: '%s'", this->c_str(), context.c_str());
	if (isComplete()) return *this;
	if (isAbsolute() && !context.isComplete()) return *this;
	if (substr(0, 2) != "..") {
		if (isAbsolute()) {
			//return normalize(context.substr(0, context.find_first_of("/", 6)) + *this);
			return normalize(context + "/" + *this);
		}
		else return (context == "" ? *this : 
                             Url(context + "/" + this->c_str()) );
	}
	else {
		bool contextWasAbsolute = context[0] == '/';
		string prefix = context.substr(0, 6) == "rdk://" ? "rdk://" : "";
		string ctxt = context.substr(prefix.size());
		vector<string> vctxt = tokenize(ctxt, "/");
		vector<string> vpath = tokenize(*this, "/");
		unsigned int c;
		for (c = 0; c < vpath.size() && vpath[c] == ".."; c++) ;
		if (c > vctxt.size() - (prefix == "" ? 0 : 1))
			throw MalformedUrlException("Too many \"..\" (context: '" 
				+ context + "', path: '" + *this + "')");
		string ret = prefix + (contextWasAbsolute ? "/" : "");
		for (size_t i = 0; i < vctxt.size() - c; i++) ret += vctxt[i] + "/";
		for (size_t i = c; i < vpath.size(); i++) ret += vpath[i] + "/";
		return ret.substr(0, ret.size() - 1);
	}
}

Url Url::decontextualize(const Url& context) const throw (MalformedUrlException)
{
	// url			context		desired result
	// rdk://a/b/c		rdk://a		b/c
	// rdk://a/b/c          /a/b            rdk://a/b/c
	// rdk://a/b/c		a/b             rdk://a/b/c
	// /b/c                 rdk://a/b	/b/c
	if (!context.isComplete()) return *this;
	if (!this->isComplete()) return *this;
	if (this->getHost() != context.getHost()) return *this;
	if (this->substr(0, context.size()) != context) return *this;
	Url u = this->substr(context.size());
	if (u[0] != '/') u = "/" + u;
	return u;
}

Url Url::getRelativeUrl(unsigned int partsToSkip) const
{
	string path = getPath();
	if (path[0] != '/') path = "/" + path;	// make it absolute
	vector<string> vc = tokenize(path, "/");
	string r = "";
	for (size_t i = partsToSkip; i < vc.size(); i++) r = r + vc[i] + "/";
	r = r.substr(0, r.size() - 1);
	return r;
}

string Url::getProtocol() const
{
	if (!isComplete()) return "";
	return substr(0, find(":"));
}

string Url::getHost() const
{
	if (!isComplete()) return "";
	size_t i = getProtocol().size() + 1;
	if (substr(i, 2) != "//") return "";
	else return substr(i + 2, find_first_of("/", i + 3) - i - 2);
}

string Url::getPath() const
{
	if (!isComplete()) return *this;
	size_t i = getProtocol().size() + 1;
	if (substr(i, 2) == "//") i += 2;
	return substr(i + getHost().size());
}

void Url::checkUrlString(const string& /*urlString*/) throw (MalformedUrlException) { }

string Url::normalize(string s)
{
	string r = "";
	string prefix = "";
	size_t p = s.find(":");
	if (p != string::npos) {
		prefix = s.substr(0, p + 1);
		if (s.substr(p + 1, 2) == "//") prefix = prefix + "//";
	}
	s = s.substr(prefix.size());
	bool slash = false;
	for (size_t i = 0; i < s.size(); i++) {
		if (s[i] == '/') slash = true;
		else {
			if (slash) { r += "/"; slash = false; }
			r += s[i];
		}
	}
	return prefix + r;
}

int Url::getLevel() const
{
	if (isRelative()) return -1;
	int l = 0;
	for (size_t i = 0; i < size(); i++) {
		if (operator[](i) == '/') l++;
	}
	if (isComplete()) l-=2;
	return l;
}

}} // namespace
