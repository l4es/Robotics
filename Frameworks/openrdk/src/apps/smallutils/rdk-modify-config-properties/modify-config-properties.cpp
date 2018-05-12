#include <rdkcore/textutils/kagetopt.h>
#include <rdkcore/textutils/textutils.h>
using namespace RDK2::TextUtils;

#include <fstream>
#include <map>
using namespace std;

int main(int argc, char** argv)
{
	KaGetOpt kgo;
	string inputConfig, paramFile, outputConfig;
	bool showHelp;
	kgo.bindParam("-in", "Input configuration file", inputConfig, "");
	kgo.bindParam("-m", "File with properties to change and their values (each line: /moduleName/propertyName=value)", paramFile, "");
	kgo.bindParam("-out", "Output configuration file", outputConfig, "");
	kgo.bindParam("-h", "Show help and exit", showHelp, false);
	kgo.parseArgs(argc, argv);
	if (showHelp || inputConfig == "" || outputConfig == "" || paramFile == "") {
		printf("Utility for modifying property values in configuration files (OpenRDK small utilities)\n");
		printf("USAGE: rdk-modify-config-properties -in <inputConfigurationFile> -m <changesFile> -out <outputConfigurationFile>\n");
		printf("%s", kgo.getHelp().c_str());
		return 0;
	}

	ifstream ifs(inputConfig.c_str());
	ifstream pfs(paramFile.c_str());
	ofstream ofs(outputConfig.c_str());
	
	if (!ifs.good()) { printf("Cannot open file '%s' for reading.\n", inputConfig.c_str()); return -1; }
	if (!pfs.good()) { printf("Cannot open file '%s' for reading.\n", paramFile.c_str()); return -1; }
	if (!ofs.good()) { printf("Cannot open file '%s' for writing.\n", outputConfig.c_str()); return -1; }
	
	string iline;
	map<string, string> propertyChanges;
	while (getline(pfs, iline)) {
		if (iline.size() && iline[iline.size()-1] == '\r') iline = iline.substr(0, iline.size()-1);
		if (iline == "") continue;
		vector<string> v = tokenize(iline, "#");
		v = tokenize(v[0], "=");
		if (v.size() == 2) {
			//printf("PROPERTY TO CHANGE: '%s' = '%s'\n", trim(v[0]).c_str(), trim(v[1]).c_str());
			propertyChanges.insert(make_pair(trim(v[0]), trim(v[1])));
		}
	}
	
	string curmodule = "";
	
	bool propsection = false;
	while (getline(ifs, iline)) {
		if (iline == "") { ofs << endl; continue; }
		if (iline[iline.size()-1] == '\r') iline = iline.substr(0, iline.size()-1);
		
		if (iline.find("<rdk:string name=\"moduleName\">") != string::npos) {
			size_t a = iline.find("<rdk:string name=\"moduleName\">");
			a += string("<rdk:string name=\"moduleName\">").size();
			size_t b = iline.find_first_of("<", a);
			curmodule = iline.substr(a, b-a);
			//printf("MODULE NAME: '%s'\n", curmodule.c_str());
			ofs << iline << endl;
		}
		else if (iline.find("<rdk:string name=\"textConfig\">") != string::npos) {
			propsection = true;
			//printf("START CONFIG\n");
			ofs << iline << endl;
		}
		else {
			if (propsection && iline.find("</rdk:string>") != string::npos) {
				//printf("END CONFIG\n");
				propsection = false;
			}
			
			if (propsection) {
				vector<string> v = tokenize(iline, "#");
				v = tokenize(v[0], "=");
				if (v.size() == 2) {
					v[0] = trim(v[0]);
					v[1] = trim(v[1]);
					map<string, string>::iterator it = propertyChanges.find("/" + curmodule + "/" + v[0]);
					if (it == propertyChanges.end()) {
						ofs << iline << endl;
					}
					else {
						ofs << v[0] << "=" << it->second << endl;
					}
				}
				else ofs << iline << endl;
			}
			else {
				ofs << iline << endl;
			}
		}
	}
}

