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
#include <fstream>
using namespace std;

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "ModuleManager"

#include "modulemanager.h"
#include "module.h"

//#define DOUT(x, args...) RDK_DEBUG_PRINTF(a, ##args)
#define DOUT(x, args...)

namespace RDK2 { namespace RAgent {

ModuleManager::ModuleManager(Repository* repository, bool isRConsole) : 
	repository(repository), isRConsole(isRConsole) { }

bool ModuleManager::instantiateModules(const ModuleConfigVector& ml)
{
	RDK_INFO_PRINTF("Instantiating all modules");
	// module instantiation from library
	for (ModuleConfigVector::const_iterator it = ml.begin(); it != ml.end(); ++it) {
		Module* m = instantiateModule(*(*it));
		if (!m) return false;
		m->loadedConfig = **it;
	}
	RDK_INFO_PRINTF("Calling initConfigurationProperties() for all modules");
	moduleListMutex.lock(HERE);
	// initConfigurationProperties()
	for (list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it) {
		if (!(*it)->initConfigurationProperties()) {
			RDK_ERROR_STREAM("Error in initConfigurationProperties() for module '" << (*it)->getModuleName() << "'");
			moduleListMutex.unlock();
			return false;
		}
	}
	RDK_INFO_PRINTF("Configuring all modules (1st step)");
	// (1st) configuration
	for (list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it) {
		(*it)->configure((*it)->loadedConfig);
	}
	RDK_INFO_PRINTF("Calling initInterfaceProperties() for all modules");
	// initInterfaceProperties()
	for (list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it) {
		if (!(*it)->initInterfaceProperties()) {
			RDK_ERROR_STREAM("Error in initInterfaceProperties() for module '" << (*it)->getModuleName() << "'");
			moduleListMutex.unlock();
			return false;
		}
	}
	RDK_INFO_PRINTF("Configuring all modules (2nd step)");
	// (2nd) configuration
	for (list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it) {
		(*it)->configure((*it)->loadedConfig, false);
	}
	moduleListMutex.unlock();
	
	return true;
}

Module* ModuleManager::instantiateModule(const ModuleConfig& cm)
{
	DOUT("Instantiating '%s' from '%s'", cm.moduleName.c_str(), cm.library.c_str());
	Module* module = Module::createFromLibrary(cm.library);
	if (!module) {
		RDK_ERROR_PRINTF("Error in module creation (library '%s')", cm.library.c_str());
		RDK_ERROR_PRINTF("Make sure you set the library path (i.e. . setenv) "
			"and the name of the library is correct");
		return 0;
	}
	module->setModuleManager(this);

	Session *session = repository->createSession(cm.moduleName,
		"Session for " + cm.moduleName + " module", "/" + cm.moduleName);
	module->setSession(session);

	Session *qtSession = repository->createSession(cm.moduleName + "_QT",
		"QT session for " + cm.moduleName + " module", "/" + cm.moduleName);
	module->setQtSession(qtSession);

	Session *asyncSession = repository->createSession(cm.moduleName + "_Async",
		"Async session for " + cm.moduleName + " module", "/" + cm.moduleName);
	module->setAsyncSession(asyncSession);

#if 0	
	if (!module->initConfigurationProperties()) {
		RDK_ERROR_PRINTF("Error in module '%s' configuration properties creation", cm.moduleName.c_str());
		return 0;
	}
	module->initWithModuleConfig(cm);

	if (!module->initInterfaceProperties()) {
		RDK_ERROR_PRINTF("Error in module '%s' configuration properties creation", cm.moduleName.c_str());
		return 0;
	}
	module->initWithModuleConfig(cm, true);
#endif
	
	addModule(module);
	return module;
}

bool ModuleManager::instantiateAdditionalModule(const ModuleConfig& cm, bool alsoStart)
{
	Module * module = instantiateModule(cm);
	if(!module) {
		RDK_ERROR_STREAM("Could not instantiate new module '" << cm.moduleName << "'");
		return false;
	}

	if (!module->initConfigurationProperties()) {
		RDK_ERROR_STREAM("Could not initConfigurationProperties() for new module '" << module->getModuleName() << "'");
		return false;
	}
	module->configure(cm);
	
	if (!module->initInterfaceProperties()) {
		RDK_ERROR_STREAM("Could not initInterfaceProperties() for new module '" << module->getModuleName() << "'");
		return false;
	}
	module->configure(cm, true);
	
	if (alsoStart) {
		if (!module->init()) {
			RDK_ERROR_STREAM("Could not init() new module '" << module->getModuleName() << "'");
			return false;
		}
		initializedModules.push_back(module);
		module->start();
	}
	
	return true;
}

// add a module in the list, the semaphore is the one where the timer signals
void ModuleManager::addModule(Module* module)
{
	moduleListMutex.lock(HERE);
	modules.push_back(module);
	moduleListMutex.unlock();
}

// call init() of every module
bool ModuleManager::initAllModules()
{
	RDK_INFO_STREAM("Initializing (init()) all modules...");
	moduleListMutex.lock(HERE);
	for (list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it) {
		DOUT("Initializing module '%s'", (*it)->getModuleName().c_str());
		if(!(*it)->init()) {
			RDK_ERROR_STREAM("Could not init module " << (*it)->getModuleName());
			RDK_ERROR_STREAM("Aborting initialization");
			return false;
		}
		initializedModules.push_back(*it);
	}
	moduleListMutex.unlock();
	RDK_INFO_STREAM("All modules initialized (init() has been called)");
	return true;
}

// instantiate a thread for each exec() of each module
void ModuleManager::startAllModules()
{
	RDK_INFO_STREAM("Starting all modules threads...");
	moduleListMutex.lock(HERE);
	for (list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it) {
		DOUT("Starting module '%s'", (*it)->getModuleName().c_str());
		(*it)->start();
	}
	moduleListMutex.unlock();
	RDK_INFO_STREAM("All modules threads started.");
}

ModuleConfigVector ModuleManager::getModuleConfigs()
{
	ModuleConfigVector ml;
	moduleListMutex.lock(HERE);
	for (list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it) {
		ml.push_back((ModuleConfig*) (*it)->createModuleConfig().clone());
	}
	moduleListMutex.unlock();
	return ml;
}

bool ModuleManager::requestExitForModule(cstr instanceName)
{
	for (list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it) {
		if ((*it)->getModuleName() == instanceName) {
			DOUT("Requesting exit for module '%s'", instanceName.c_str());
			(*it)->requestExit();
			DOUT("Requested exit for module '%s'", instanceName.c_str());
			return true;
		}
	}
	return false;
}

bool ModuleManager::deleteModule(cstr instanceName)
{
	//moduleListMutex.lock(HERE);	// FIXME
	for (list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it) {
		if ((*it)->getModuleName() == instanceName) {
			if (find(initializedModules.begin(), initializedModules.end(), *it) != initializedModules.end()) {
				DOUT("Requesting exit for module '%s'", instanceName.c_str());
				(*it)->requestExit();
				DOUT("Waiting for module '%s' to exit", (*it)->getModuleName().c_str());
				if ((*it)->threadId) pthread_join((*it)->threadId, 0);
				DOUT("Calling cleanup for module '%s'", (*it)->getModuleName().c_str());
				(*it)->cleanup();
				initializedModules.erase(find(initializedModules.begin(), initializedModules.end(), *it));
			}
			DOUT("Deleting session %x", (*it)->session);
			delete (*it)->session;
			DOUT("Deleting qtSession %x", (*it)->qtSession);
			delete (*it)->qtSession;
			DOUT("Deleting asyncSession %x", (*it)->asyncSession);
			delete (*it)->asyncSession;
			delete (*it);
			modules.erase(it);
			return true;
		}
	}
	//moduleListMutex.unlock();
	RDK_ERROR_PRINTF("Unknown module '%s'", instanceName.c_str());
	return false;
}

// signal something to the module (requestExit()) and wait for each exec() to exit, then call cleanup()
void ModuleManager::endAllModules()
{
	moduleListMutex.lock(HERE);
	for (list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it) {
		DOUT("Requesting exit for module '%s'", (*it)->getModuleName().c_str());
		if (find(initializedModules.begin(), initializedModules.end(), *it) != initializedModules.end()) {
			(*it)->requestExit();
		}
	}
	moduleListMutex.unlock();

	moduleListMutex.lock(HERE);
	for (list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it) {
		DOUT("Waiting for module '%s' to exit", (*it)->getModuleName().c_str());
		pthread_join((*it)->threadId, 0);
		DOUT("Module '%s' exited", (*it)->getModuleName().c_str());
	}
	moduleListMutex.unlock();
}

void ModuleManager::cleanupAllModules()
{
	moduleListMutex.lock(HERE);
	for (list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it) {
		DOUT("Calling cleanup for module '%s'", (*it)->getModuleName().c_str());
		if (find(initializedModules.begin(), initializedModules.end(), *it) != initializedModules.end()) {
			(*it)->cleanup();
			initializedModules.erase(find(initializedModules.begin(), initializedModules.end(), *it));
		}
		delete (*it)->session;
		delete (*it)->qtSession;
		delete (*it)->asyncSession;
		delete (*it);
	}
	moduleListMutex.unlock();
}

#if 0
void ModuleManager::moduleSafeCleanup(Module *module)
{
	list<const Module*>::iterator it;
	bool found = false;
	// Find the module in our list
	for (it = initializedModules.begin(); 
	     ((it != initializedModules.end()) && !found);
	     it++) {
		if ((*it) == module) {
			found = true;
			module->cleanup();
		}
	}
	initializedModules.remove(module);			
}
#endif

}} // namespace
