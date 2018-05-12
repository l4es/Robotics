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

#include <cxxabi.h>
#include <iostream>
#include "demangle.h"
#include <cstdlib>

namespace RDK2 { namespace Demangle {

/** X -> X     (..::)*X->X	*/	
std::string deleteNamespaces(const std::string& s) { 
	size_t i = s.find_last_of(":");
	if(i==std::string::npos) return s;
	
	return s.substr(i+1);
}

/** Vecchia implementazione */
void rdemangle(const char * sym, char*buf) ;

std::string demangle(const char*symbol) {
	if(0) {
		char buf[512];
		rdemangle(symbol, buf);
		return std::string(buf);
	} else {
		int     status;
		char *realname_c = abi::__cxa_demangle(symbol, 0, 0, &status);
		std::string realname(realname_c); free(realname_c);
		std::string after =  deleteNamespaces(realname);
		//if (realname.find("<") != std::string::npos) std::cout << "demangle: " << realname << " -> " << after << std::endl;
		return after;
	}
}


struct global {
	const char * symbol;
	char * buf;
	int idx;
	int ibuf;
};

void try_type(global&g);

void rdemangle(const char * sym, char*buf) {
	global g;
	g.symbol=sym;
	g.ibuf=0;
	g.idx=0;
	g.buf = buf;
	try_type(g);
	g.buf[g.ibuf]=0;
}


void simple_class(global&g) {
	int nc=0;
	char c;
	for (;(c=g.symbol[g.idx])>='0'&&c<='9';g.idx++)
		nc=nc*10+(c-'0');
	for (int i=0;i<nc;i++)
		g.buf[g.ibuf++]=g.symbol[g.idx++];
}

void native_type(global&g) {
	char c=g.symbol[g.idx++];
	if (c=='U') {
		for (const char * us="unsigned ";*us;us++)
			g.buf[g.ibuf++]=*us;
		c=g.symbol[g.idx++];
	}
	const char * nm="unknown";
	switch(c) {
		case 'i':
			nm="int";
			break;
		case 'l':
			nm="long";
			break;
		case 's':
			nm="short";
			break;
		case 'c':
			nm="char";
			break;
		case 'x':
			nm="long long";
			break;
		case 'f':
			nm="float";
			break;
		case 'd':
			nm="double";
			break;
		case 'b':
			nm="bool";
			break;
		case 'w':
			nm="wchar_t";
			break;
	}
	for (;*nm;nm++)
		g.buf[g.ibuf++]=*nm;
}

void pointer(global&g) {
	g.idx++;
	try_type(g);
	g.buf[g.ibuf++]='*';
}

void reference(global&g) {
	g.idx++;
	try_type(g);
	g.buf[g.ibuf++]='&';
}

void qualified_class(global&g) {
	g.idx++;
	char c=g.symbol[g.idx++];
	int nc=0;
	if (c>='0'||c<='9') nc=c-'0';
	else
		while ((c=g.symbol[g.idx++])>='0'&&c<='9') nc=nc*10+(c-'0');
	for (int i=0;i<nc;i++) {
		if (i>0) { g.buf[g.ibuf++]=':'; g.buf[g.ibuf++]=':'; }
		try_type(g);
	}
}
		
void template_class(global&g) {
	g.idx++;
	try_type(g);
	int nc=0;
	char c;
	for (;(c=g.symbol[g.idx])>='0'&&c<='9';g.idx++) nc=nc*10+(c-'0');
	g.buf[g.ibuf++]='(';
	for (int i=0;i<nc;i++) {
		if (i>0) g.buf[g.ibuf++]=',';
		c=g.symbol[g.idx];
		if (c=='Z') {
			g.idx++;
			for (const char * nm="class ";*nm;nm++) g.buf[g.ibuf++]=*nm;
			try_type(g);
		} else {
			try_type(g);
			g.buf[g.ibuf++]='=';
			for (;(c=g.symbol[g.idx])>='0'&&c<='9';g.idx++) g.buf[g.ibuf++]=c;
		}
	}
	g.buf[g.ibuf++]=')';
}

void try_type(global&g) {
	char c=g.symbol[g.idx];
	if (c>='0'&&c<='9') {
		simple_class(g);
		return;
	}
	if (c=='i'||c=='l'||c=='s'||c=='c'||c=='x'||c=='U'||c=='f'||c=='d'||c=='b'||c=='w') {
		native_type(g);
		return;
	}
	if (c=='P') {
		pointer(g);
		return;
	}
	if (c=='R') {
		reference(g);
		return;
	}
	if (c=='Q') {
		qualified_class(g);
		return;
	}
	if (c=='t') {
		template_class(g);
		return;
	}
	return;
}

}} // namespace RDK2::Demangle
