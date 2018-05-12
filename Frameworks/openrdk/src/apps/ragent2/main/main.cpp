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

#include "ragent.h"

#include <rdkcore/profiling/profiling.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/object/objectmanager.h>
#define LOGGING_MODULE "RAgent"

#include <signal.h>
#include <fstream>
#include <rdkcore/config.h>

using namespace RDK2::RAgent;
using namespace RDK2::Profiling;

RAgent ragent;

void trapSignal(int c);
void editing();
void writeRandomQuote();

int ragent_main(int argc, char** argv, bool rconsole)
{
	if (rconsole) ragent.setRConsole(true);

	bool editConfigFile = false;
	if (!ragent.parseArgs(argc, argv, editConfigFile)) {
		writeRandomQuote();
		return -1;
	}
	
	if (ragent.profilingActive) {
		Profiler::open("profiling.log");
		Profiler::addCustomLine("RAGENT", "START");
		RDK_INFO_PRINTF("Profiler active");
	}

	RDK_INFO_PRINTF("Loading configuration");
	if (!ragent.loadConfiguration()) {
		return -1;
	}

	ragent.createMainObjects();
	if (!ragent.instantiateModules()) {
		return -1;
	}

	if (editConfigFile) {
		editing();
		return 0;
	}

	RDK_INFO_PRINTF("Starting RAgent '%s'", ragent.getAgentName().c_str());

	if (!ragent.start()) {
		RDK_ERROR_STREAM("Could not initialize all modules.");
		return -1;
	}
	signal(SIGINT, trapSignal);
	
	ragent.quitSemaphore.wait();
	//sleep(5);	// FIXME for debugging

	RDK_INFO_STREAM("Ending...");
	ragent.end();
	RDK_INFO_STREAM("Saving configuration..");
	ragent.saveConfiguration();
	RDK_INFO_STREAM("Cleaning up..");
	ragent.cleanup();
	
	RDK_INFO_PRINTF("RAgent '%s' cleanly closed", ragent.getAgentName().c_str());
	
	if (ragent.profilingActive) {
		Profiler::addCustomLine("RAGENT", "END");
	}
	
	RDK2::Meta::freePrototypeDb();	// XXX veramente lo mettiamo qui?
	writeRandomQuote();
	
	return 0;
}

// Reads aphorisms separated by newlines
vector<string> loadQuotes() {
	ifstream aphstream((string() + OpenRDK_RESOURCES + "/../docs/aphorisms.txt").c_str());
	
	vector<string> aphs;
	string line; string aph = "";
	while (getline(aphstream, line)) {
		if(line.length()==0 && aph.length()>0) {
			aphs.push_back(aph);
			aph = "";
		} else {
			aph = aph + (aph.length()>0 ? "\n":"") + line;
		}
	}
	
	if(aph.length())
		aphs.push_back(aph);
	
	return aphs;
}

void writeRandomQuote() {
	vector<string> aphs = loadQuotes();
	if (aphs.empty())
	{
		printf("\n%s\n", "No one will see me... :-)");
	}
	else
	{
		srand(time(0));
		string quote = aphs[rand() % aphs.size()];
		printf("\n%s\n", quote.c_str());
	}
}

void trapSignal(int c)
{
	signal(SIGINT, trapSignal);
	RDK_INFO_PRINTF("Trapped CTRL-C (%d), re-trapped and trying to close", c);
	static int count = 0;
	ragent.quitSemaphore.signal();
	count++;
	int debianMultiplier = 1;
#ifdef DEBIAN
	debianMultiplier = 11;
#endif
	if (count == 2 * debianMultiplier) {
		RDK_INFO_PRINTF("Next CTRL-C will make me exit uncleanly");
	}
	else if (count >= 3 * debianMultiplier) {
		RDK_ERROR_PRINTF("Unclean exit... things are not so good");
		exit(-1);
	}
}

void editing()
{
	// FIXME spostare in RAgent
	RDK_INFO_PRINTF("Editing configuration file '%s'...", ragent.parsedArgs["agentConfigFilename"].c_str());
	char c;
	do {
		cout << endl << endl << "The configuration file now contains the following modules:" << endl;
		int i = 1;
		for (ModuleConfigVector::iterator it = ragent.ragentConfig.moduleConfigs.begin();
		it != ragent.ragentConfig.moduleConfigs.end(); ++it) {
			cout << i++ << " " << (*it)->moduleName << " (library " << (*it)->library << ")" << endl;
		}
		cout << endl;
		cout << "Type 'a' to add a module to the configuration file" << endl;
		cout << "Type 'c' to start the configuration helper" << endl;
		cout << "Type 'd <modulename>' to delete a module" << endl;
		cout << "Type 's <filename>' to save this configuration to file <filename>" << endl;
		cout << "Type 's @' to save this configuration file to the same file" << endl;
		cout << "Type 'x' or 'q' to exit" << endl;
		cout << "Choose: ";
		cin >> c;
		if (c == 'a') {
			cout << endl << "Modules currently available:" << endl;

			string scmd = string() + "ls ";

#ifdef MACOSX
			string ldpath = "DYLD_LIBRARY_PATH";
#else
			string ldpath = "LD_LIBRARY_PATH";
#endif
			char * paths = getenv(ldpath.c_str());
			if (paths)
			{
				string path_list(paths);
				size_t pos = 0;
				size_t len = path_list.length();
				while (pos < len)
				{
					size_t epos = path_list.find_first_of(':',pos);
					if (epos == string::npos)
					{
						epos = len;
					}
					scmd += path_list.substr(pos,epos - pos) + " ";
					pos = epos+1;
				}
			}
			else
			{
				scmd = "echo \'" + ldpath + " not specified, impossible to list available modules!\' # ";
			}

			string libnamescmd = scmd + " | grep rdkr.m | sed -e 's@.*rdkr.m_\\([^.]*\\)module.*@\\1@' | sort -u";
			int dummy = system(libnamescmd.c_str()); dummy = dummy;
			cout << "Type the name of the library of the module to add (e.g. 'navigator'): ";

			string libraryName;
			cin >> libraryName;

			FILE *pf;
			const int DATA_SIZE = 128;
			char prefix[DATA_SIZE];
			char suffix[DATA_SIZE];

			// prefix
			pf = popen(string(scmd + " | grep rdk | grep _" + libraryName + "m | sed 's@.*\\(rdkr.m\\)_.*@\\1_@' | tr -d '\\n'").c_str(),"r");
			if(!pf)
			{
				cerr << "Could not retrieve module type, aborting." << endl;
				return;
			}
			char * d = fgets(prefix, DATA_SIZE , pf); d = d;
			if (pclose(pf) != 0)
				cerr << " Error: Failed to close command stream." << endl; 

			// suffix
			pf = popen(string(scmd + " | grep rdk | grep _" + libraryName + "m | sed 's@.*" + libraryName + "\\(.*\\)\\..*@\\1@' | tr -d '\\n'").c_str(),"r"); 
			if(!pf)
			{
				cerr << "Could not retrieve module type, aborting." << endl;
				return;
			}
			d = fgets(suffix, DATA_SIZE , pf); d = d;
			if (pclose(pf) != 0)
				cerr << " Error: Failed to close command stream." << endl; 

			libraryName = string(prefix) + libraryName + string(suffix);
			cout << "Type the name of the module in the configuration file (e.g. 'theBestNavigator'): ";
			string moduleName;
			cin >> moduleName;
			ModuleConfig* moduleConfig = new ModuleConfig();
			moduleConfig->library = libraryName;
			moduleConfig->moduleName = moduleName;
			if (ragent.moduleManager->instantiateAdditionalModule(*moduleConfig, false)) {
				ragent.ragentConfig.moduleConfigs.push_back(moduleConfig);
			}
		}
		else if (c == 'd') {
			string which;
			cin >> which;
			if (ragent.moduleManager->deleteModule(which)) {
				for (ModuleConfigVector::iterator it = ragent.ragentConfig.moduleConfigs.begin();
				it != ragent.ragentConfig.moduleConfigs.end(); ++it) {
					if ((*it)->moduleName == which) {
						ragent.ragentConfig.moduleConfigs.erase(it, true);
						break;
					}
				}
			}
		}
		else if (c == 's') {
			string filename;
			cin >> filename;
			if (filename != "@") {
				if (!RDK2::TextUtils::endsWith(filename, ".config")) {
					RDK_ERROR_STREAM("The configuration filename must end with '.config'");
					continue;
				}
				else ragent.parsedArgs["agentConfigFilename"] = filename;
			}
			ragent.saveConfiguration();
			RDK_INFO_STREAM("Saved configuration file to '" << 
				ragent.parsedArgs["agentConfigFilename"] << "'");
			RDK2::Meta::freePrototypeDb();	// XXX veramente lo mettiamo qui?
		}
		else if (c == 'c') {
			
			cout << "Configuration helper" << cout;
			//vector<Url> urls = 
		}
	} while (c != 'x' && c != 'q');
}
