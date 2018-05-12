#include <fstream>
#include <rdkcore/network/tcpsocket.h>
#include <sstream>
#include <cstdlib>

using namespace Network;
using namespace std;

int main(int argc, char** argv)
{
	if (argc != 3) {
		printf("USAGE: simpletcpclient <address> <port>\n\n");
		return -1;
	}
	TCPSocket sock(argv[1], atoi(argv[2]));
	string cmd = "";
	cout << "RDK Simple TCP client" << endl << "type command and press return, 'help' to get a list of commands from the agent, 'quit' to exit" << endl << endl;
	while (cmd != "quit") {
		cout << "\nType a command: ";
		getline(cin, cmd);
		if (cmd == "quit") break;
		//cout << "Sending: '" << cmd << "'" << endl;
		sock << cmd << endl;
		string answer;
		getline(sock, answer);
		//cout << "Answer: '" << answer << "'" << endl;
		cout << answer << endl;
		istringstream iss(answer);
		string answertag;
		iss >> answertag;
		if (cmd.substr(0, 8) == "getImage") {
			if (answertag == "IMAGE-VALUE-OF:") {
				string pname, imgformat;
				size_t imgsize;
				iss >> pname >> imgformat >> imgsize;
				cout << "Downloading image, size = " << imgsize << " bytes, format = " << imgformat << endl;
				char* buf = new char[imgsize];
				sock.read(buf, imgsize);
				string fname = "rdkimage." + imgformat;
				ofstream ofs(fname.c_str());
				ofs.write(buf, imgsize);
				delete[] buf;
				cout << "Saved as '" << fname << "'" << endl;
				int ret = system(("display " + fname).c_str());
				fprintf(stderr,"command '%s' returned %d",("display " + fname).c_str(),ret);
			}
		}
		else if (cmd.substr(0, 6) == "getMap") {
			if (answertag == "MAP-VALUE-OF:") {
				string pname, imgformat;
				double mapx, mapy, mapres;
				size_t imgsize;
				iss >> pname >> imgformat >> mapx >> mapy >> mapres >> imgsize;
				cout << "Downloading (map) image, x = " << mapx << ", y = " << mapy
					<< ", res = " << mapres << " pixel/m, size = " << imgsize
					<< " bytes, format = " << imgformat << endl;
				char* buf = new char[imgsize];
				sock.read(buf, imgsize);
				string fname = "rdkmap." + imgformat;
				ofstream ofs(fname.c_str());
				ofs.write(buf, imgsize);
				delete[] buf;
				cout << "Saved as '" << fname << "'" << endl;
				int ret = system(("display " + fname).c_str());
				fprintf(stderr,"command '%s' returned %d",("display " + fname).c_str(),ret);
			}
		}
		else if (cmd.substr(0, 12) == "propertyList" || cmd.substr(0, 4) == "help") {
			int linestoread = 0;
			if (answertag == "LINES:") {
				iss >> linestoread;
			}
			else {
				cout << "ERROR: Malformed answer '" << answertag << "'" << endl;
			}
			for (int i = 0; i < linestoread; i++) {
				getline(sock, answer);
				cout << answer << endl;
			}
		}
	}
	return 0;
}

