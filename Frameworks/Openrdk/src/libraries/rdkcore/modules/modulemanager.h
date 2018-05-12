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

#ifndef RDK2_MODULEMANAGER
#define RDK2_MODULEMANAGER

#include <list>

#include <rdkcore/posixconstructs/posixsem.h>
#include <rdkcore/textutils/textutils.h>
#include <rdkcore/repository/repository.h>
#include <rdkcore/repository/session.h>
#include <rdkcore/container/container.h>
#include <rdkcore/modules_config/ragentconfig.h>

namespace RDK2 { namespace RAgent {

class Module;

using namespace std;
using namespace RDK2::TextUtils;
using namespace RDK2::RepositoryNS;

class ModuleManager {
public:
	/// Constructor
	ModuleManager(Repository* repository, bool isRConsole = false);

	/// Instantiates all modules
	bool instantiateModules(const ModuleConfigVector&);

	/// Instantiates an additional (i.e. after global initialization) module; this will also init and start the module
	bool instantiateAdditionalModule(const ModuleConfig& cm, bool alsoStart = true);

	/// Calls init*() of every module
	bool initAllModules();

	/// Instantiates a thread for each module exec()
	void startAllModules();

	/// Creates the module configurations
	ModuleConfigVector getModuleConfigs();

	/// Signal something to the module (requestEnd()) and wait for each exec() to exit
	void endAllModules();

	/// Calls cleanup() of each module
	void cleanupAllModules();

	bool requestExitForModule(cstr instanceName);

	/// Ends, calls cleanup and removes the module
	bool deleteModule(cstr instanceName);

	PosixMutex moduleListMutex;
		list<Module*> modules;

private:
	Repository* repository;

	bool isRConsole;

	// adds a module in the list
	void addModule(Module* module);

	/// Returns 0 on error
	Module* instantiateModule(const ModuleConfig&cm);

	/// Vector of init()-ed modules.  The other must not be cleanup()-ed
	list<const Module*> initializedModules;

#if 0
	/// Call cleanup() on module if it was previously init()-ed
	void moduleSafeCleanup(Module *module);
#endif
};

}} // namespaces

#endif
