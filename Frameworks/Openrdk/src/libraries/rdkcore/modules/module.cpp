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

#include <dlfcn.h>

using namespace std;

#include <rdkcore/textutils/textutils.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "Module"

#include "module.h"

namespace RDK2 { namespace RAgent {

Module* Module::createFromLibrary(cstr libraryName)
{
	string libName = RDK2::TextUtils::normalizeLibraryName(libraryName);
	Module* (*factoryFunction)();

	dlerror();
	void* handle = dlopen(libName.c_str(), RTLD_LAZY);
	if (!handle) {
		RDK_ERROR_PRINTF("Cannot dlopen() library '%s'; dlerror() = '%s'", libName.c_str(), dlerror());
		return 0;
	}

	dlerror();
	factoryFunction = (Module*(*)())dlsym(handle, MODULE_FACTORY_FUNCTION);
	if (!factoryFunction || (long) factoryFunction == -1) {
		RDK_ERROR_PRINTF("Cannot find factory function '%s' in library '%s'; dlerror() = '%s'",
			MODULE_FACTORY_FUNCTION, libName.c_str(), dlerror());
		return 0;
	}
	
	return (*factoryFunction)();
}

Module::~Module()
{
	//RDK_DEBUG_STREAM("Deleting module '" << getModuleName()<< "'");
}

ModuleConfig Module::createModuleConfig()
{
	ModuleConfig cm;
	cm.moduleName = moduleName;
	cm.library = library;
	
	//RDK_DEBUG_PRINTF("Creating module configuration for '%s'", moduleName.c_str());

	Repository* repository = session->getRepository();
	// FIXME non per nome, ma per sessione!
	vector<Url> props = repository->getLocalPropertiesStartingWith(session->getUrlContext()+"/");
	for (size_t i = 0; i < props.size(); i++) {
		SESSION_TRY_START(session)
		Url relativeUrl = props[i].getRelativeUrl(1);
		cm.descriptions.insert(make_pair(relativeUrl, session->getPropertyDescription(relativeUrl)));
		
		//RDK_DEBUG_PRINTF("%s:", relativeUrl.c_str());
		
		const PropertyDef& defPd = session->getDefaultPropertyDef(relativeUrl);
		const PropertyDef& pd = session->getPropertyDef(relativeUrl);
		if (defPd.isPersistent()) {
			//RDK_DEBUG_PRINTF("Def: PERSISTENT");
			if (pd.isVolatile()) cm.userOptions[relativeUrl].insert("VOLATILE");
			if (pd.isKeepThis()) cm.userOptions[relativeUrl].insert("KEEP_THIS");
		}
		else if (defPd.isKeepThis()) {
			//RDK_DEBUG_PRINTF("Def: KEEP_THIS");
			if (pd.isVolatile()) cm.userOptions[relativeUrl].insert("VOLATILE");
			if (pd.isPersistent()) cm.userOptions[relativeUrl].insert("PERSISTENT");
		}
		else if (defPd.isVolatile()) {
			//RDK_DEBUG_PRINTF("Def: VOLATILE");
			if (pd.isPersistent()) cm.userOptions[relativeUrl].insert("PERSISTENT");
			if (pd.isKeepThis()) cm.userOptions[relativeUrl].insert("KEEP_THIS");
		}

		for (set<string>::iterator it = cm.userOptions[relativeUrl].begin(); it != cm.userOptions[relativeUrl].end(); ++it) {
			//RDK_DEBUG_PRINTF("  %s", it->c_str());
		}
	
		if (pd.isVolatile()) {
			if (!pd.isLink()) {
				// it is not a link
				cm.textObjConfigs.insert(make_pair("#" + relativeUrl, ""));
				cm.descriptions[relativeUrl] = "(volatile) " + cm.descriptions[relativeUrl];
			}
			else {
				// it is a link
				cm.textObjConfigs.insert(make_pair(relativeUrl, "@" + pd.getLinkTo()));
			}
		}
		else if (pd.isPersistent()) {
			if (!pd.isLink()) {
				// it is not a link
				RDK2::Object* obj = session->getObjectClone(relativeUrl);
				if (obj) {
					if (obj->hasStringRepresentation()) {
						// NON-NULL OBJECT WITH STRING REPRESENTATION
						cm.textObjConfigs.insert(make_pair(relativeUrl, obj->getStringRepresentation()));
						delete obj;
					}
					else {
						// NON-NULL OBJECT WITHOUT STRING REPRESENTATION
						cm.textObjConfigs.insert(make_pair("#" + relativeUrl, ""));
						cm.objConfigs.push_back(new Pair(relativeUrl, obj));
						cm.descriptions[relativeUrl] = "(see XML) " + cm.descriptions[relativeUrl];
					}
				}
				else {
					// NULL OBJECT
					cm.textObjConfigs.insert(make_pair("#" + relativeUrl, ""));
					cm.descriptions[relativeUrl] = "(NULL) " + cm.descriptions[relativeUrl];
				}
			}
			else {
				// it is a link
				cm.textObjConfigs.insert(make_pair(relativeUrl, "@" + pd.getLinkTo()));
			}
		}
		else if (pd.isKeepThis()) {
			// put in the configuration the old value, regardless of what there is now in the property
			bool oldObjectFound = false;
			// look for the old object in the old textual configuration
			map<string, string>::iterator it = loadedConfig.textObjConfigs.find(relativeUrl);
			if (it != loadedConfig.textObjConfigs.end()) {
				oldObjectFound = true;
				cm.textObjConfigs.insert(make_pair(relativeUrl, it->second));
			}
			if (!oldObjectFound) {
				// if it was not in the old textual configuration, maybe it is in the xml config
				for (PropertyList::const_iterator it = cm.objConfigs.begin();
				it != cm.objConfigs.end(); ++it) {
					if ((*it)->url == relativeUrl) {
						cm.textObjConfigs.insert(make_pair("#" + relativeUrl, ""));
						cm.objConfigs.push_back(new Pair(relativeUrl, (*it)->value->clone()));
						cm.descriptions[relativeUrl] = "(see XML)" + cm.descriptions[relativeUrl];
						oldObjectFound = true;
						break;
					}
				}
			}
		}
		SESSION_END_CATCH_TERMINATE(session)
	}

	return cm;
}

void Module::configure(const ModuleConfig& cm, bool configureOnlyPendingProperties)
{
	if (!configureOnlyPendingProperties) {
		moduleName = cm.moduleName;
		library = cm.library;
	}
	
	// options
	for (map<string, set<string> >::const_iterator it = cm.userOptions.begin();
	it != cm.userOptions.end(); ++it) {
		const set<string>& propUserOpts = it->second;
		SESSION_TRY_START(session)
		string pclass = session->getPropertyDef(it->first).getObjectClassName();
		PropertyDef defPd = session->getDefaultPropertyDef(it->first);
		// persistence options
		if (defPd.isPersistent()) {
			//RDK_DEBUG_PRINTF("Is persistent");
			if (propUserOpts.find("KEEP_THIS") != propUserOpts.end())
				session->setKeepThis(it->first);
			else if (propUserOpts.find("VOLATILE") != propUserOpts.end())
				session->setVolatile(it->first);
		}
		else if (defPd.isKeepThis()) {
			//RDK_DEBUG_PRINTF("Is keep this");
			if (propUserOpts.find("PERSISTENT") != propUserOpts.end())
				session->setPersistent(it->first);
			else if (propUserOpts.find("VOLATILE") != propUserOpts.end())
				session->setVolatile(it->first);
		}
		else if (defPd.isVolatile()) {
			//RDK_DEBUG_PRINTF("Is volatile");
			if (propUserOpts.find("PERSISTENT") != propUserOpts.end()) {
				//RDK_DEBUG_PRINTF("Mananagggaaaaa");
				session->setPersistent(it->first);
				//PropertyDef aapd = session->getPropertyDef(it->first);
				//RDK_DEBUG_PRINTF("%d", aapd.isPersistent());
			}
			else if (propUserOpts.find("KEEP_THIS") != propUserOpts.end())
				session->setKeepThis(it->first);
		}
		// FIXME other options
		SESSION_END_CATCH_TERMINATE(session)
	}
	
	// for override message
	set<string> xmlConfigured;

	// xml object description configuration
	for (PropertyList::const_iterator it = cm.objConfigs.begin();
	it != cm.objConfigs.end(); ++it) {
		SESSION_TRY_START(session)
			cstr url = (*it)->url;
			if (!configureOnlyPendingProperties
			|| pendingConfigurableProperties.find(url) != pendingConfigurableProperties.end()) {
				PropertyDef pd = session->getPropertyDef(url);
				if (pd.isPersistent() || pd.isKeepThis()) {
					RDK2::Object* clone = (*it)->value->clone();
					session->setObject(url, clone);
					xmlConfigured.insert(url);
				}
			}
			session->end();
		SESSION_CATCH(session)
		session->terminate();
		if (!configureOnlyPendingProperties)
			pendingConfigurableProperties.insert((*it)->url);
		else
			RDK_ERROR_PRINTF(e.what());
		}
	}

	// text configuration
	for (map<string, string>::const_iterator it = cm.textObjConfigs.begin();
	it != cm.textObjConfigs.end(); ++it) {
		SESSION_TRY_START(session)
			if (!configureOnlyPendingProperties
			|| pendingConfigurableProperties.find(it->first) != pendingConfigurableProperties.end()) {
				PropertyDef pd = session->getPropertyDef(it->first);
				if (it->second != "") {
					if (xmlConfigured.find(it->first) != xmlConfigured.end()) {
						RDK_ERROR_PRINTF("XML config value for property '%s' has been overriden "
							"by textual config value (this is not an error)", it->first.c_str());
					}
					if (pd.isPersistent() || pd.isKeepThis() || it->second[0] == '@') {
						// FIXME DC: assolutamente da mettere altrove, ad esempio in session
						session->getRepository()->setPropertyValueFromTextConfig(it->first, it->second,
							session->getUrlContext(), session);
					}
				}
			}
			session->end();
			SESSION_CATCH(session)
			session->terminate();
			if (!configureOnlyPendingProperties)
				pendingConfigurableProperties.insert(it->first);
			else
				RDK_ERROR_PRINTF(e.what());
		}
	}
	
	if (!configureOnlyPendingProperties)
		loadedConfig = cm;		// save this initial configuration for later use (e.g. saving KEEP_THIS values)
}

void Module::start() {
	pthread_create(&threadId, 0, (void*(*)(void*)) Module::thread_exec, this);
}

void* Module::thread_exec(Module* m) {
	RDK_DEBUG_STREAM("Starting thread for '" << m->getModuleName() << "' ptr: " << m);
	
	try {
		m->exec();
	} catch(SessionException e) {
		m->session->terminate();
		RDK_ERROR_STREAM("Module '" <<  m->getModuleName() << "' let this exception pass through:");
		RDK_ERROR_STREAM(e.what());
	} /*catch(std::exception e) {
		m->session->terminate();
		RDK_ERROR_STREAM("Module '" << m->getModuleName() << "' let this exception pass through:");
		RDK_ERROR_STREAM(e.what() << " (this name is mangled!)");
	} catch(...) {
		m->session->terminate();
		RDK_ERROR_STREAM("Module '" << m->getModuleName() << "' let an exception pass through.");
	}*/
	
	RDK_DEBUG_STREAM("Module '" << m->getModuleName() << "' has exited");
	return 0;
}

void Module::requestExit()
{
	exiting = true;
	session->exiting = true;
	qtSession->exiting = true;
	asyncSession->exiting = true;
	exitRequested();
	session->wakeUp();
	qtSession->wakeUp();
	asyncSession->wakeUp();
}

}} // namespace
