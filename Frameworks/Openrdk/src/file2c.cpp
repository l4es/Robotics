#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char**argv) {
	if(argc != 4) {
		cerr << argv[0] << ": Usage is " << argv[0] << " <input> <output> <variable name> " << endl;
		return -1;
	}
	
	ifstream ifs(argv[1]);
	ofstream ofs(argv[2]);
	ofs << endl << "extern \"C\" const char  " << argv[3] << "[] = { ";
	char c; int count=0;
	while(ifs.read(&c,1)) {
		if(count!=0) ofs << ", ";
		ofs << ((int)c);
		if((count+1) % 32 == 0) ofs << endl;
		count++;
	}
	ofs << ",0}; " << endl << endl;
	ofs.close();
	ifs.close();
}


