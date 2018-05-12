#include <rdkcore/serialization_xml/xmlreader.h>
#include <rdkcore/serialization_xml/xmlwriter.h>
using namespace RDK2::Serialization::Xml;

#include <rdkcore/modules_config/ragentconfig.h>
using namespace RDK2::RAgent;

#include <rdkcore/filesystem/filesystem.h>
using namespace RDK2::Filesystem;

#include <fstream>
using namespace std;

int main(int argc, char** argv)
{
	if (argc != 3) {
		printf("rdk-remove-rqm <input config> <output config>\n");
		exit(0);
	}

	XmlReader xr;
	RAgentConfig* rac = dynamic_cast<RAgentConfig*>(xr.deserialize(fsReadFile(argv[1])));
	if (!rac) {
		printf("Something wrong...\n");
		exit(-1);
	}

	for (ModuleConfigVector::iterator it = rac->moduleConfigs.begin(); it != rac->moduleConfigs.end(); ) {
		printf("Module '%s', library '%s' ", (*it)->moduleName.c_str(), (*it)->library.c_str());
		if ((*it)->library.substr(0, 6) == "rdkrqm") {
			it = rac->moduleConfigs.erase(it, true);
			printf("is a RConsoleQt module, removing\n");
		}
		else {
			++it;
			printf("is a common RAgent module, keeping\n");
		}
	}	

	XmlWriter xw(true);
	string newconfig = xw.serialize(true, rac);

	ofstream ofs(argv[2]);
	ofs << newconfig;
	ofs.close();

	return 0;
}

