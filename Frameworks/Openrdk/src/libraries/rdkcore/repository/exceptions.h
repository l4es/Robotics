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

#ifndef RDK2_SESSION_EXCEPTIONS
#define RDK2_SESSION_EXCEPTIONS

#include <exception>
#include <stdexcept>

#define DECLARE_EXCEPTION(SUB, SUPER) \
	struct SUB: public SUPER { \
		explicit SUB(const std::string&e):SUPER(e) { } \
}; 

namespace RDK2 { namespace RepositoryNS {

	/** Base for exception generated by this class. */
	DECLARE_EXCEPTION(SessionException,	std::runtime_error);
	
	/** Can't find a property with the specified name. */
	DECLARE_EXCEPTION(NoSuchProperty,	SessionException);
	
	/** You are asking for a property which is not set 
	    and doesn't have a default value. */
	DECLARE_EXCEPTION(ValueNotSet,		SessionException);

	/** You are asking an object of a type and the property 
	    contains objects of another type. */
	DECLARE_EXCEPTION(WrongType,		SessionException);

	/** Other exeptions */
	DECLARE_EXCEPTION(InvalidOperation,	SessionException);

	DECLARE_EXCEPTION(MalformedUrlException, SessionException);

}} // namespaces

#endif