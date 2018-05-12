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

#ifndef H_REPOSITORY_URL
#define H_REPOSITORY_URL

#include <string>
#include <rdkcore/exceptions/exceptions.h>

namespace RDK2 { namespace RepositoryNS {

/** An url is composed by (http://www.ietf.org/rfc/rfc1738.txt):

<scheme>://<user>:<password>@<host>:<port>/<url-path>

scheme: "rdk" (always)
user, password: never used
port : never used, managed at another level (i.e.: implicit)
host : a string
url-path : a string

i.e. in the rdk we will always have:

rdk://<host>/<url-path>

in which <host> is the name of the repository and <url-path> is the path of the property in that repository

when you specify a url as a string, you should consider that

* a string without a starting "/" is a relative path (relative to the current context)
* a string with a starting "/" is absolute
* a string can begin with one ore more "../", the meaning is the same as for filesystem paths

examples (current host is "foo", current context is "bar/rap"):

something          -> rdk://foo/bar/rap/something
mike/something     -> rdk://foo/bar/rap/mike/something
../something       -> rdk://foo/bar/something
/something         -> rdk://foo/something
../../something    -> rdk://foo/something
../../../something -> <error>
rdk://another/foo  -> rdk://another/foo

*/

using namespace std;

class Url : public string {
public:
	Url(const string& urlString) throw (MalformedUrlException);
	Url(const char*) throw (MalformedUrlException);
	Url& operator=(const Url& url) throw (MalformedUrlException);

	/// Returns the url contextualized in the @param context ; if this url is complete,
	/// return this url itself, regardless of the context
	/// @param context the context
	Url contextualize(const Url& context) const throw (MalformedUrlException);
	
	/// Returns the url decontextualized using the @param context ; if the context
	/// contains the beginning part of the url, this will be stripped; the url will be made absolute
	/// if only the protocol and host part are stripped, otherwise it will be made relative
	Url decontextualize(const Url& context) const throw (MalformedUrlException);
	
	/// Returns the protocol part of the url (i.e., "rdk", "http", etc.), without any "/" or ":"
	string getProtocol() const;
	
	/// Returns the host of the url, if it is complete; otherwise it returns ""
	string getHost() const;

	/// If the url is complete, returns the corresponding absolute url (without the host part),
	/// otherwise it returns the url unchanged
	string getPath() const;

	/// If the url is absolute or complete, strips the protocol and host part and @param n "directories"
	Url getRelativeUrl(unsigned int partsToSkip) const;

	/// Returns true if this url begins with @param prefix
	/// FIXME using slashes to divide the url (maybe isInContext() is a better name)
	inline bool startsWith(const Url& prefix) const
		{ return substr(0, prefix.size()) == prefix; }

	/// Url is complete when it has also the host part (e.g. it begins with "rdk://")
	inline bool isComplete() const { return find(":") != string::npos; }

	/// Url is relative if it isn't complete and doesn't begin with a "/"
	inline bool isRelative() const { return !isComplete() && operator[](0) != '/'; }

	/// Url is absolute if it isn't complete and begins with a "/"
	inline bool isAbsolute() const { return !isComplete() && operator[](0) == '/'; }

	/// DEPRECATED
	int getLevel() const;

private:
	static void checkUrlString(const string& urlString) throw (MalformedUrlException);
	static string normalize(string s);
};

typedef const Url& CUrl;

}} // end namespace

#endif
