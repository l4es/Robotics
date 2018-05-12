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

#ifndef H_RAGENT
#define H_RAGENT

#include <string>
#include <map>
#include <rdkcore/modules/modulemanager.h>
#include <rdkcore/posixconstructs/posixsem.h>
#include <rdkcore/repository/repository.h>

#include <rdkcore/modules_config/ragentconfig.h>

#include "agentcmdreceiver.h"

namespace RDK2 { namespace RAgent {

using namespace std;
using namespace RDK2::RepositoryNS;

class RAgent {
public:
	RAgent() : moduleManager(0), repository(0), isRConsole(false) { }
	~RAgent() { }

	bool parseArgs(int argc, char** argv, bool& editConfigFile);
	void createMainObjects();
	bool loadConfiguration();
	void loadObjectsFromFiles();
	bool instantiateModules();
	bool start();
	void end();
	void saveConfiguration();
	void saveObjectsToFiles();
	void cleanup();

	void setRConsole(bool isRConsole) { this->isRConsole = isRConsole; }

	PosixConstructs::PosixSemaphore quitSemaphore;

	string getAgentName() { return ragentConfig.agentName; }

	RAgentConfig ragentConfig;
	map<string, string> parsedArgs;
	ModuleManager* moduleManager;

	bool profilingActive;

private:
	Repository* repository;
	AgentCmdReceiver* agentCmdReceiver;
	bool isRConsole;
};

}} // namespace

#endif
