#include <rdkcore/network/tcpsocket.h>
#include <rdkcore/textutils/kagetopt.h>

#include <unistd.h>

#include <fstream>
#include <sstream>
#include <cstdlib>

using namespace Network;
using namespace std;
using namespace RDK2::TextUtils;

void skipChars(string& l, string chars)
{
	size_t c = l.find_first_not_of(chars);
	if (c == string::npos) l = "";
	else l = l.substr(c);
}

string readUntilChar(string& l, string chars)
{
	string r;
	size_t c = l.find_first_of(chars);
	if (c == string::npos) { r = l; l = ""; return r; }
	else { r = l.substr(0, c); l = l.substr(c+1); return r; }
}

int main(int argc, char** argv)
{
	KaGetOpt opt;
	opt.optionString("-a", "USARSim server address");
	opt.optionInt("-p", "USARSim server port", 3000);
	opt.optionString("-f", "UWB filename to send to the server");
	opt.optionInt("-r", "Delay in seconds to reload and re-send the file (0 means send only once and stay connected, -1 means only one and exit)", 0);
	opt.optionDouble("-tx", "Translation along the x axis", 0);
	opt.optionDouble("-ty", "Translation along the y axis", 0);
	opt.optionDouble("-tz", "Translation along the z axis", 0);
	opt.optionBool("-d", "Show debug information (what is sent to the server)", false);
	opt.optionInt("-id", "ID for the first object", 0);
	opt.optionBool("-noclear", "Do not clear the environment before sending the world", false);
	opt.optionBool("-h", "Show this help and exit", false);
	string address, filename;
	int port, delay, firstId;
	double tx, ty, tz;
	bool showHelp, showDebug, noClear;
	opt.bind("-a", address);
	opt.bind("-p", port);
	opt.bind("-f", filename);
	opt.bind("-r", delay);
	opt.bind("-tx", tx); opt.bind("-ty", ty); opt.bind("-tz", tz);
	opt.bind("-d", showDebug);
	opt.bind("-id", firstId);
	opt.bind("-noclear", noClear);
	opt.bind("-h", showHelp);
	if (!opt.parseArgs(argc, argv)) showHelp = true;
	if (address == "") showHelp = true;
	printf("USARSim World Builder file sender (OpenRDK smallutil)\n");
	if (showHelp) {
		printf("%s", opt.getHelp().c_str());
		return 0;
	}

	printf("Connecting to USARSim server at %s:%d...", address.c_str(), port);

	TCPSocket sock;	
	try {
		sock.connect(address, port);
	}
	catch (const exception& e) {
		printf("failed\n");
		return -1;
	}
	printf("OK\n");

	#define SOCK_SEND(s) if (showDebug) cout << s; sock << s << flush;

	SOCK_SEND("INIT {ClassName USARBot.WorldController} {Name WC} {Location 0.1,0.1,-0.1}\r\n");

	cout << "Clearing environment...\n";
	if (!noClear) { SOCK_SEND("CONTROL {Type KillAll}\r\n"); }

	if (tx != 0.0 || ty != 0.0 || tz != 0.0) cout << "Translating the environment (USARSim coords: " << tx << ", " << ty << ", " << tz << ")\n";

	int i = firstId;	// object ID
	if (filename != "") {
		cout << "Sending environment...";
		if (showDebug) cout << endl;
		while (true) {
			fstream ifs(filename.c_str());
			if (!ifs.good()) {
				printf("Cannot open file '%s'\n", filename.c_str());
			}
			else {
				string l;
				while (getline(ifs, l)) {
					skipChars(l, " ");
					string model = readUntilChar(l, " (");
					skipChars(l, " (");
					double lx = atof(readUntilChar(l, ",").c_str()); skipChars(l, " ,");
					double ly = atof(readUntilChar(l, ",").c_str()); skipChars(l, " ,");
					double lz = atof(readUntilChar(l, ")").c_str());
					readUntilChar(l, "(");
					skipChars(l, " (");
					double rx = atof(readUntilChar(l, ",").c_str()); skipChars(l, " ,");
					double ry = atof(readUntilChar(l, ",").c_str()); skipChars(l, " ,");
					double rz = atof(readUntilChar(l, ")").c_str());
					if (model != "") {
						ostringstream oss;
						oss << "CONTROL {Type Create} {ClassName USARModels."<<model<<"} "
							<< "{Name wcObj"<<i++<<"} "
							<< "{Location "<<(lx+tx)<<","<<(ly+ty)<<","<<(lz+tz)<<"} "
							<< "{Rotation "<<rx<<","<<ry<<","<<rz<<"}\r\n";
						SOCK_SEND(oss.str());
					}
				}
			}
			if (!showDebug) cout << "." << flush;

			if (delay > 0) usleep(delay * 1e6);
			else if (delay == -1) break;
			else while (true) usleep (10 * 1e6);
			SOCK_SEND("CONTROL {Type KillAll}\r\n");
		}
	}

	return 0;
}

