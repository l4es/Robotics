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

#ifndef H_MODULE
#define H_MODULE
/**
 * @file
 *
 * @brief This file contains the Module class and other very important stuff.
 */

#include <string>
#include <pthread.h>
#include <rdkcore/textutils/textutils.h>
#include <rdkcore/repository/session.h>

#include "modulemanager.h"

namespace RDK2 { namespace RAgent {

using namespace std;
using namespace RDK2::RepositoryNS;

#if 0
/*
	Avvio secondo module manager (pseudocodice).
*/
	Repository r;

	// leggi configurazione, crea istanze moduli, crea session
	...

	for (m in modules) {
		// Prima di caricare la configurazione, devi creare le proprietà
		m.initConfigurationProperties();
		// Carica la configurazione
		m.session.readConfiguration(m.session.getString("config"));
		// E' possibile che la creazione di alcune altre properties
		// dipenda dalla configurazione (esempio: se non voglio pubblicare
		// il percorso del pathplanner, non creo la property).
		// L'esempio è cretino ma è bene avere qui un altro hook
		// per il modulo
		m.initOtherProperties();
	}

	// ora tutte le proprietà sono a posto
	// e quindi i moduli possono farsi negli init() le sottoscrizioni
	// anche incrociate

	for (m in modules) {
		// può essere lungo
		if(!m.init()) {
			abort();
		}
	}
#endif

class ModuleManager;

/**
 * This is the base class of each RDK module.
 */
class Module : public Session::SessionEventObserver {

public:
	Module(): exiting(false), session(0), qtSession(0), asyncSession(0)/*, description("Undocumented module"), handleName(""), canHandleAClass(false)*/, threadId(0) { }

///// Interface:

	/**
	 * Declare configuration properties.
	 *
	 * This method should have a very short execution time.
	 *
	 * ASSUMPTIONS: this module can't make any assumption.
	 */
	virtual bool initConfigurationProperties() { return true; }

	/**
	 * Declare properties and queues that interface to other modules.
	 *
	 * ASSUMPTIONS: configuration was loaded, but no assumption
	 * can be made about other modules.
	 *
	 */
	virtual bool initInterfaceProperties() { return true; }

	/// Inizializzazione risorse (apri file, socket, ecc)
	/// ASSUNZIONI: gli altri moduli sono stati caricati e le loro
	/// proprietà create. I moduli elencati prima di te hanno fatto init()
	/// con successo.
	/**
	 * Initialize resources (open files, sockets, etc.)
	 *
	 * ASSUMPTIONS: other modules are loaded and their properties
	 * are created. Modules listed before this have succesfully
	 * init()-ed.
	 */
	virtual bool init() { return true; }

	/// Instantiates module thread that will call exec()
	void start();

	/// ASSUMPTION: all modules init()-ed succesfully
	virtual void exec() = 0;

	/**
	 * This method should follow what's written in the wiki page
	 * http://sied.dis.uniroma1.it/private/doku.php?id=rdk:robot_modules
	 */
	virtual void asyncAgentCmd(cstr /*command*/) { }

	/**
	 * This method stes the description of the module
	 */
	virtual inline void setDescription(cstr desc) { description = desc; }

	/**
	 * This method returns the description of the module
	 */
	virtual const cstr getDescription() { return description; }

	//Not used yet
	//virtual const cstr getHandleName() { return handleName; }

	//virtual void setIsHandler(bool ishandler) { canHandleAClass = ishandler; }

	//virtual bool isHandler() { return canHandleAClass; }

	/// when this function exits, the module manager can safely call pthread_join
	/// the user should implement this only if the module has something to be closed
	/// before the thread is joined (e.g.: close sockets)
	virtual void exitRequested() { }

	virtual void cleanup() {};

	// this function is called by the main thread: it set exiting variables,
	// calls user's exitRequested and then signals all sessions
	void requestExit();
	virtual ~Module();

	string getModuleName() { return moduleName; }
//	string getClassName() { return className; }
//	void setInstanceName(cstr instanceName) { this->instanceName = instanceName; }

	void setSession(Session* session) { this->session = session; }
	void setQtSession(Session* qtSession) { this->qtSession = qtSession; }
	void setAsyncSession(Session* asyncSession) { this->asyncSession = asyncSession; }

	static Module* createFromLibrary(cstr libraryName);

	void configure(const ModuleConfig& cm, bool configureOnlyPendingProperties = false);
	
	ModuleConfig createModuleConfig();
	ModuleConfig loadedConfig;

protected:
	volatile bool exiting;
	Session* session;
	Session* qtSession;
	Session* asyncSession;

	set<string> pendingConfigurableProperties;
	
	string moduleName, library;
	string description;
	// temporary patch
	// needed for rconsoleqt viewers
	//string handleName;
	//bool canHandleAClass;

	ModuleManager* moduleManager;
	ModuleManager* getModuleManager() { return moduleManager; }
	void setModuleManager(ModuleManager* mm) { moduleManager = mm; }

private:
	static void* thread_exec(Module* m);
	pthread_t threadId;

friend class ModuleManager;
};

}} // namespaces

#define MODULE_FACTORY(MyModule) extern "C" RDK2::RAgent::Module* create_module() { return new MyModule(); }
#define MODULE_FACTORY_FUNCTION "create_module"

#endif
