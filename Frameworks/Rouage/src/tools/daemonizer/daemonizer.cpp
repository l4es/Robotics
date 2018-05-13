#include "daemonizer.h"
#include <stdio.h>

Daemonizer::Daemonizer(){
}

int Daemonizer::launch(){
  /* detach from 1st gen */
  switch (fork())
    {
    case 0 : break;
    case -1: return -1;
    default: _exit(0);
    }
  if (setsid() < 0) return -1;

  /* detach from 2nd gen */
  switch (fork())
    {
    case 0 : break;
    case -1: return -1;
    default: _exit(0);
    }


  /* change to a secure place */
  chdir("/");

  /* close all file descriptors used by ancestors */
  /*
  int fd=0;
  int maxfd=sysconf(_SC_OPEN_MAX);
  fprintf(stderr,"%d df to check\n",maxfd);
  while (fd < maxfd){
    close(fd++);
  }
  fprintf(stderr,"ok\n");
  */  
  daemon_pid=getpid();

  return 0;
}

int Daemonizer::get_pid(){
  return daemon_pid;
}

Daemonizer::~Daemonizer(){
}
