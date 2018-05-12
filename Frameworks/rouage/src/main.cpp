#include "robot.h";

int main(int argc,char **argv){
  // TODO: parse options
  char *filename;

  // TODO: add logging (with quiet or verbose options) 

  // build a empty robot, passing command line options
  Robot *I=new Robot(int argc,char **argv);

  // filename is a xml file, with the full module tree. build() is a bison parser
  I.build(filename);

  // for every module who needs an init step
  I.init_modules();

  // for every module running as daemon
  I.launch_daemons();

  // for every module requiring running in loop
  while (I.loop());

  // Kill all the daemon-modules
  I.shutdown();
  
  // clean memory and resources
  I.clean();

  return 0;
}
