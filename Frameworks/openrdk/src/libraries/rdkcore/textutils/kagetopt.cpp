
#include "kagetopt.h"

#include <vector>
#include <string>
#include <map>
#include <cstdlib>

namespace RDK2 { namespace TextUtils {

using namespace std;

void KaGetOpt::Option::parseOptionString(const string& os)
{
	string curOpt = "";
	for (size_t i = 0; i < os.size() + 1; i++) {
		if (i == os.size() || os[i] == ' ') {
			if (curOpt != "") options.push_back(curOpt);
			curOpt = "";
		}
		else curOpt += string(1, os[i]);
	}
}

KaGetOpt::Option::Option(const string& options, const string& description, char type) :
	description(description), type(type), hasDefaultValue(false), hasValue(false), boundVar(0)
{
	parseOptionString(options);
}

void KaGetOpt::Option::setValue(const string& value)
{
	this->hasValue = true;
	this->value = value;
	if (boundVar) {
		if (this->type == 's') *(string*)boundVar = value;
		else if (this->type == 'i') *(int*)boundVar = atoi(value.c_str());
		else if (this->type == 'd') *(double*)boundVar = atof(value.c_str());
		else if (this->type == 'b') *(bool*)boundVar = (value == "true" || value == "1");
	}
}

KaGetOpt::Option::Option(const string& options, const string& description, char type, const string& defaultValue) :
	description(description), type(type), hasDefaultValue(true), defaultValue(defaultValue), hasValue(false), boundVar(0)
{
	parseOptionString(options);
}

void KaGetOpt::optionString(const string& option, const string& description)
{
	options.push_back(Option(option, description, 's'));
}

void KaGetOpt::optionString(const string& option, const string& description, const string& defaultValue)
{
	options.push_back(Option(option, description, 's', defaultValue));
}

void KaGetOpt::optionInt(const string& option, const string& description)
{
	options.push_back(Option(option, description, 'i'));
}

void KaGetOpt::optionInt(const string& option, const string& description, int defaultValue)
{
	options.push_back(Option(option, description, 'i', toString(defaultValue)));
}

void KaGetOpt::optionDouble(const string& option, const string& description)
{
	options.push_back(Option(option, description, 'd'));
}

void KaGetOpt::optionDouble(const string& option, const string& description, double defaultValue)
{
	options.push_back(Option(option, description, 'd', toString(defaultValue)));
}

void KaGetOpt::optionBool(const string& option, const string& description)
{
	options.push_back(Option(option, description, 'b'));
}

void KaGetOpt::optionBool(const string& option, const string& description, bool defaultValue)
{
	options.push_back(Option(option, description, 'b', toString(defaultValue)));
}

string KaGetOpt::getHelp()
{
	string ret = "Options: \n";
	for (size_t i = 0; i < options.size(); i++) {
		ret += "  ";
		size_t optsize = 0;
		for (size_t j = 0; j < options[i].options.size(); j++) {
			string n = (j > 0 ? " , " : "") + options[i].options[j];
			ret += n;
			optsize += n.size();
		}
		ret += string((optsize >= 25 ? 1 : 25 - optsize), ' ');
		ret += options[i].description;
		if (options[i].type == 'b' || options[i].hasDefaultValue) {
			ret += " (";
			if (options[i].hasDefaultValue) ret += "default: " + options[i].defaultValue;
			ret += ")";
		}
		ret += "\n";
	}
	return ret;
}

bool KaGetOpt::getIt(const string& option, string& value)
{
	for (size_t i = 0; i < options.size(); i++) {
		for (size_t j = 0; j < options[i].options.size(); j++) {
			if (option == options[i].options[j]) {
				Option& opt = options[i];
				if (!opt.hasValue && !opt.hasDefaultValue) {
					lastError = "No value for " + option;
					return false;
				}
				else {
					value = opt.hasValue ? opt.value : opt.defaultValue;
					return true;
				}
			}
		}
	}
	lastError = "Unknown option " + option;
	return false;
}

KaGetOpt::Option* KaGetOpt::findOption(const string& option)
{
	for (size_t i = 0; i < options.size(); i++) {
		for (size_t j = 0; j < options[i].options.size(); j++) {
			if (option == options[i].options[j]) {
				return &(options[i]);
			}
		}
	}
	return 0;
}

bool KaGetOpt::bind(const string& option, string& s)
{
	Option* opt = findOption(option);
	if (!opt) { lastError = "Unknown option " + option; return false; }
	if (opt->type != 's') { lastError = "Option " + option + " is not a string option"; return false; }
	opt->boundVar = &s;
	if (opt->hasDefaultValue) s = opt->defaultValue;
	return true;
}

bool KaGetOpt::bind(const string& option, int& i)
{
	Option* opt = findOption(option);
	if (!opt) { lastError = "Unknown option " + option; return false; }
	if (opt->type != 'i') { lastError = "Option " + option + " is not an int option"; return false; }
	opt->boundVar = &i;
	if (opt->hasDefaultValue) i = atoi(opt->defaultValue.c_str());
	return true;
}

bool KaGetOpt::bind(const string& option, double& d)
{
	Option* opt = findOption(option);
	if (!opt) { lastError = "Unknown option " + option; return false; }
	if (opt->type != 'd') { lastError = "Option " + option + " is not a double option"; return false; }
	opt->boundVar = &d;
	if (opt->hasDefaultValue) d = atof(opt->defaultValue.c_str());
	return true;
}

bool KaGetOpt::bind(const string& option, bool& b)
{
	Option* opt = findOption(option);
	if (!opt) { lastError = "Unknown option " + option; return false; }
	if (opt->type != 'b') { lastError = "Option " + option + " is not a bool option"; return false; }
	opt->boundVar = &b;
	if (opt->hasDefaultValue) b = (opt->defaultValue == "true" || opt->defaultValue == "1");
	return true;
}

bool KaGetOpt::parseArgs(int argc, char** argv)
{
	Option* pendingOption = 0;
	string strippedArgs = "";	
	for (int i = 1; i < argc; i++) {
		if (argv[i][0] == '-') {
			if (pendingOption && (pendingOption->type == 'i' || pendingOption->type == 'd')) {
				pendingOption->setValue(string(argv[i]));
				pendingOption = 0;
			}
			else {
				if (pendingOption) {
					if (pendingOption->type == 'b') {
						pendingOption->setValue("true");
						pendingOption = 0;
					}
					else {
						lastError = "No value given for option " + string(argv[i]);
						return false;
					}
				}
				if (string(argv[i]) == "--") {	// stop parsing arguments
					for (int j = i + 1; j < argc; j++) strippedArgs += string(argv[j]) + " ";
					break;
				}
				else {
					pendingOption = findOption(string(argv[i]));
					if (!pendingOption) {
						lastError = "Unknown option " + string(argv[i]);
						return false;
					}
				}
			}
		}
		else {
			if (pendingOption) {
				pendingOption->setValue(string(argv[i]));
				pendingOption = 0;
			}
			else {
				strippedArgs += string(argv[i]) + " ";
			}
		}
	}
	if (pendingOption) {
		if (pendingOption->type == 'b') pendingOption->setValue("true");
		else {
			lastError = "No value given for option " + pendingOption->options[0];
			return false;
		}
	}
	return true;
}

bool KaGetOpt::getString(const string& option, string& value)
{
	return getIt(option, value);
}

bool KaGetOpt::getInt(const string& option, int& value)
{
	string v;
	if (!getIt(option, v)) return false;
	value = atoi(v.c_str());
	return true;
}

bool KaGetOpt::getDouble(const string& option, double& value)
{
	string v;
	if (!getIt(option, v)) return false;
	value = atof(v.c_str());
	return true;
}

bool KaGetOpt::getBool(const string& option, bool& value)
{
	string v;
	if (!getIt(option, v)) return false;
	value = (v == "true");
	return true;
}

}}

