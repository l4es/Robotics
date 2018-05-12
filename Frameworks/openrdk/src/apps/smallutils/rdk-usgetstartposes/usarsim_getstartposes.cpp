#include <stdio.h>

#include <rdkcore/network/tcpsocket.h>
using namespace Network;

#include <rdkcore/textutils/textutils.h>
using namespace RDK2::TextUtils;

#include <rdkcore/geometry/angle.h>
using namespace RDK2::Geometry;

#include <string>
#include <vector>
#include <sys/types.h>
#include <fstream>
#include <cstring>
#include <cstdlib>

using namespace std;


string substring(string s, size_t start, size_t end)
{
	return s.substr(start, end-start);
}

string getusarline(istream& ist)
{
	char c;
	ostringstream oss;
	while (true) {
		if (!ist.get(c)) return "";
		if (c == '\n') return oss.str();
		if (c == '\r') { ist.get(c); return oss.str(); }
		oss << c;
	}
}

int main(int argc, char** argv)
{
	if (!(argc >= 2 && argc <= 4)) {
		printf("USAGE: rdk-usgetstartposes [-wsp] <server-ip-address> [<server-port>]\n");
		return -1;
	}
	
	const char* host = 0;
	int port = 3000;
	bool writeStartPoses = false;
	
	if (strcmp(argv[1], "-pu") == 0) {
		host = argv[2];
		if (argc == 4) port = atoi(argv[3]);
		writeStartPoses = true;
	}
	else {
		host = argv[1];
		if (argc == 3) port = atoi(argv[2]);
	}

	TCPSocket sck(InetAddress(host, port));
	printf("Connected to server at %s:%d\n", host, port);

	string ln = getusarline(sck);

	//printf("Received: '%s'\n", ln.c_str());

	sck << "GETSTARTPOSES\r\n";
	sck.flush();

	#define PERR { printf("Malformed answer from the server (%s)\n", ln.c_str()); return -1; }

	ofstream ofsUs;
	if (writeStartPoses) {
		ofsUs.open("startposes.txt");
	}

	if ((ln = getusarline(sck)) != "") {
		//printf("Received: '%s'\n", ln.c_str());
		if (ln.substr(0, 3) != "NFO") PERR
		ln = ln.substr(4);
		string a = substring(ln, ln.find_first_of("{")+1, ln.find_first_of("}"));
		printf("%s\n", a.c_str());
		if (a.substr(0, 10) != "StartPoses") PERR
		int b = atoi(a.substr(11).c_str());
		ln = ln.substr(a.size()+2);
		a = substring(ln, ln.find_first_of("{")+1, ln.find_first_of("}"));
		istringstream iss(a);
		printf("Poses in the map (RDK reference system: meters and degrees)\n");
		for (size_t i = 0; i < (uint) b; i++) {
			string name, position, orientation;
			iss >> name >> position >> orientation;
			double p1, p2, p3, a1, a2, a3;
			vector<string> v = tokenize(position, ",");
			p1 = atof(v[0].c_str());
			p2 = -atof(v[1].c_str());
			p3 = -atof(v[2].c_str());
			v = tokenize(orientation, ",");
			a1 = atof(v[0].c_str());
			a2 = -atof(v[1].c_str());
			a3 = -atof(v[2].c_str());
			printf("\t%s: (x, y, z) = %.2f,%.2f,%.2f; (roll, pitch, yaw) = %.2f,%.2f,%.2f\n", name.c_str(), 
				p1, p2, p3, rad2deg(a1), rad2deg(a2), rad2deg(a3));
			if (writeStartPoses) {
				ofsUs << name << "," << p1 << "," << -p2 << "," << a3 << endl;
			}
		}
	}

	return 0;
}

