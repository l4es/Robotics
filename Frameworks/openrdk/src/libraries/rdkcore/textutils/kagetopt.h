#ifndef KA_KAGETOPT_H
#define KA_KAGETOPT_H

#include <string>
#include <vector>
#include <sstream>

namespace RDK2 { namespace TextUtils {

using namespace std;

template<typename T>
string toString(const T& v) { ostringstream oss; oss << v; return oss.str(); }

template<>
string toString(const bool& v) { return (v ? "true" : "false"); }

class KaGetOpt {
public:
	/// *structors
	KaGetOpt() { }

	/// params declaration
	void optionString(const string& option, const string& description);
	void optionString(const string& option, const string& description, const string& defaultValue);
	void optionInt(const string& option, const string& description);
	void optionInt(const string& option, const string& description, int defaultValue);
	void optionDouble(const string& option, const string& description);
	void optionDouble(const string& option, const string& description, double defaultValue);
	void optionBool(const string& option, const string& description);
	void optionBool(const string& option, const string& description, bool defaultValue);

	/// automatic variable binding during parseArgs
	bool bind(const string& option, string& s);
	bool bind(const string& option, int& i);
	bool bind(const string& option, double& d);
	bool bind(const string& option, bool& b);

	/// params declaration and bind
	inline bool bindParam(const string& option, const string& description, string& s, const string& defaultValue) { optionString(option, description, defaultValue); return bind(option, s); } 
	inline bool bindParam(const string& option, const string& description, string& s) { optionString(option, description); return bind(option, s); }
	inline bool bindParam(const string& option, const string& description, int& i, int defaultValue) { optionInt(option, description, defaultValue); return bind(option, i); }
	inline bool bindParam(const string& option, const string& description, int& i) { optionInt(option, description); return bind(option, i); }
	inline bool bindParam(const string& option, const string& description, double& d, double defaultValue) { optionDouble(option, description, defaultValue); return bind(option, d); }
	inline bool bindParam(const string& option, const string& description, double& d) { optionDouble(option, description); return bind(option, d); }
	inline bool bindParam(const string& option, const string& description, bool& b, bool defaultValue) { optionBool(option, description, defaultValue); return bind(option, b); }
	inline bool bindParam(const string& option, const string& description, bool& b) { optionBool(option, description); return bind(option, b); }

	// parses the command line
	bool parseArgs(int argc, char** argv);

	/// returns the string without parameters
	inline string getStrippedArgs() { return strippedArgs; }

	/// returns help message
	string getHelp();

	/// returns the string explaining the last error occurred
	inline string getLastError() { return lastError; }

	/// gets option values
	bool getString(const string& option, string& value);
	bool getInt(const string& option, int& value);
	bool getDouble(const string& option, double& value);
	bool getBool(const string& option, bool& value);
	
private:
	struct Option {
		Option(const string& options, const string& description, char type);
		Option(const string& options, const string& description, char type, const string& defaultValue);
		void parseOptionString(const string& os);
		void setValue(const string& val);
		vector<string> options;
		string description;
		char type;
		bool hasDefaultValue;
		string defaultValue;
		bool hasValue;
		string value;
		void* boundVar;		
	};
	bool getIt(const string& s, string& r);
	Option* findOption(const string& option);
	vector<Option> options;
	string strippedArgs;
	string lastError;
};

}} // namespace

#endif
