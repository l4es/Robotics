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

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RAgent"

#include <rdkcore/profiling/profiling.h>
#include <rdkcore/serialization_xml/xmlreader.h>
#include <rdkcore/serialization_xml/xmlwriter.h>
#include <rdkcore/filesystem/filesystem.h>
#include <fstream>

#include "ragent.h"
#include "agentcmdreceiver.h"

namespace RDK2 { namespace RAgent {

using namespace std;
using namespace RDK2::Serialization::Xml;
using namespace RDK2::Filesystem;
using namespace RDK2::Profiling;

bool RAgent::parseArgs(int argc, char** argv, bool& editConfigFile)
{
	if (getenv("RDK_AGENT_NAME") != NULL && string(getenv("RDK_AGENT_NAME")) != "" && ragentConfig.agentName == "") {
		RDK_WARNING_PRINTF("Agent name set by environment variable RDK_AGENT_NAME: '%s'", getenv("RDK_AGENT_NAME"));
		sleep(1);
		ragentConfig.agentName = string(getenv("RDK_AGENT_NAME"));
	}

	bool agentConfigFound = false;
	editConfigFile = false;
	for (int i = 0; i < argc; ) {
		string argvi = string(argv[i]);
		if (argvi == "-c" || argvi == "--config") {
			parsedArgs["agentConfigFilename"] = argv[i+1];
			i+=2;
			agentConfigFound = true;
		}
		else if (argvi == "-n" || argvi == "--name" || argvi == "--agentName") {
			parsedArgs["agentName"] = argv[i+1];
			if (ragentConfig.agentName != "") {
				RDK_WARNING_PRINTF("Agent name provided by command line overrided environment variable: the agent name is '%s'", parsedArgs["agentName"].c_str());
				sleep(1);
			}
			ragentConfig.agentName = argv[i+1];
			i+=2;
		}
		else if (argvi == "--editConfigFile" || argvi == "-e") {
			parsedArgs["agentConfigFilename"] = argv[i+1];
			i+=2;
			editConfigFile = true;
			agentConfigFound = true;
		}
		else if (argvi == "--profiling") {
			profilingActive = true;
			i++;
		}
		else i++;
	}


	if (!agentConfigFound) {
		RDK_ERROR_PRINTF("To start an ragent use: ragent2 -c <agentConfigFilename>");
		RDK_ERROR_PRINTF("if you want to edit a configuration file use: ragent2 -e <agentConfigFilename>");
	}
	return agentConfigFound;
}

void RAgent::saveObjectsToFiles()
{
	//for (size_t i = 0; i < ragentConfig.moduleConfigs.size(); i++) {
	//  ModuleConfig* mc = ragentConfig.moduleConfigs[i]; // FIXME Is this useful? Please avoid unused variables
	//}
}

void RAgent::loadObjectsFromFiles()
{
	//for (size_t i = 0; i < ragentConfig.moduleConfigs.size(); i++) {
	//  ModuleConfig* mc = ragentConfig.moduleConfigs[i]; // FIXME Is this useful? Please avoid unused variables
	//  // XXX il fatto che il file venga parsato due volte forse non � bene (vedi
	//  // anche module.initWithModuleConfig (rdk2/modules/module.cpp)
		
	//}
}

void RAgent::saveConfiguration() {
	if (profilingActive) Profiler::addCustomLine("RAGENT", "savingConfiguration");
	
	ragentConfig.moduleConfigs = moduleManager->getModuleConfigs();

	try {
		XmlWriter xw(true);
		string xmlString = xw.serialize(true, &ragentConfig);
		string configFile = parsedArgs["agentConfigFilename"];
		ofstream ofs(configFile.c_str());
		ofs << xmlString << endl;
		saveObjectsToFiles();
	}
	catch (RDK2::ReadingException&e) {
		RDK_ERROR_STREAM(e.what());
	}
	catch (std::exception& e) {
		RDK_ERROR_STREAM(e.what());
	}
	catch (...) {
		RDK_ERROR_STREAM("Unexpected exception");
	}
}

bool RAgent::loadConfiguration()
{
	if (profilingActive) Profiler::addCustomLine("RAGENT", "loadingConfiguration");
	try {
		string configFilename = parsedArgs["agentConfigFilename"];
		string xmlString = fsReadFile(configFilename);
		XmlReader xmlReader;
		xmlReader.deserialize(xmlString, &ragentConfig);
		loadObjectsFromFiles();
	}
	catch (RDK2::ReadingException&e) {
		RDK_ERROR_STREAM(e.what());
		return false;
	}
	catch (std::exception& e) {
		RDK_ERROR_STREAM(e.what());
		return false;
	}
	catch (...) {
		RDK_ERROR_STREAM("Unexpected exception");
		return false;
	}
	return true;
}

void RAgent::createMainObjects()
{
	repository = new Repository(ragentConfig.agentName);
	moduleManager = new ModuleManager(repository, isRConsole);
	agentCmdReceiver = new AgentCmdReceiver(repository, moduleManager);
}

bool RAgent::instantiateModules()
{
	if(!moduleManager->instantiateModules(ragentConfig.moduleConfigs)) {
		RDK_ERROR_STREAM("Could not instantiate all modules");
		return false;
	}
	repository->saveAllDefaultPropertyDefs();
	return true;
}

bool RAgent::start()
{
	if (!repository->loadYellowPages(ragentConfig.ypFilename)) {
		return false;
	}

	if (profilingActive) Profiler::addCustomLine("RAGENT", "initializingModules");
	if(!moduleManager->initAllModules()) {
		RDK_ERROR_STREAM("Could not init() all modules, exiting.");
		return false;
	}
	
	if (profilingActive) Profiler::addCustomLine("RAGENT", "startingRepositoryThreads");
	repository->startRepositoryThreads();

	agentCmdReceiver->init();
	agentCmdReceiver->start();

	if (profilingActive) Profiler::addCustomLine("RAGENT", "startingModules");
	moduleManager->startAllModules();

	return true;
}

void RAgent::end()
{
	if (profilingActive) Profiler::addCustomLine("RAGENT", "ending");
	repository->endRepositoryThreads();
	moduleManager->endAllModules();
	agentCmdReceiver->end();
}

void RAgent::cleanup()
{
	if (profilingActive) Profiler::addCustomLine("RAGENT", "cleaningUpModules");
	moduleManager->cleanupAllModules();
	delete agentCmdReceiver;
	delete moduleManager;
	delete repository;
}

}} // namespace
