#include "robot.h"

Robot::Robot(){
  daemonizer=new Daemonizer();
  daemonizer->launch();
}

int Robot::loop(){
  for (int i=0;i<10000;i++){
  };
  return 0;
}

Robot::~Robot(){
}


