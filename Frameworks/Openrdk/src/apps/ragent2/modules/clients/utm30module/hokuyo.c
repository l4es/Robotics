#include "hokuyo.h"
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <termios.h>

// added for serial communication
//#include <carmen/carmenserial.h>
// end added for serial communication

#if defined(__GNUC_GNU_INLINE__) || defined(__GNUC_STDC_INLINE__)
#define inline inline __attribute__ ((gnu_inline))
#endif

//parses an int of x bytes in the hokyo format
unsigned int parseInt(int bytes, char** s){
  int i;
  char* b=*s;
  unsigned int ret=0;
  int j=bytes-1;
  for (i=0; i<bytes;){
    if (*b==0||*b=='\n'){
      *s=0;
      return 0;
    }
    if (*(b+1)=='\n'){ //check for a wrapped line
      b++;
    } else {
      unsigned char c=*b-0x30;
      ret+=((unsigned int ) c)<<(6*j);
      i++;
      j--;
    }
    b++;
  }
  *s=b;
  return ret;
}


//skips a line
char* skipLine(char* buf){
  while (*buf!=0 && *buf!='\n')
    buf++;
  return (*buf=='\n')?buf+1:0;
}

//parses a reading response
void hokuyo_parseReading(HokuyoRangeReading* r, char* buffer, int remission)
{
  char* s=buffer;
  int expectedStatus=0;
  if (s[0]=='M')
    expectedStatus=99;
  if (s[0]=='C')
    expectedStatus=00;

  /*fprintf(stderr, "debug buffer %c%c\n", s[0], s[1]);*/
  
  int beamBytes=0;
  if (s[1]=='D' || s[1] == 'E') beamBytes=3;
  if (s[0]=='C') beamBytes=3;
  
  if (! beamBytes || ! expectedStatus){
    fprintf(stderr, "Invalid return packet, cannot parse reading\n");
    r->status=-1;
    return;
  }
  s+=2;
  char v[5];
  v[4]=0;
  strncpy(v,s,4); r->startStep=atoi(v); s+=4;
  strncpy(v,s,4); r->endStep=atoi(v);   s+=4;
  v[2]=0; strncpy(v,s,2); r->clusterCount=atoi(v);
  
  s=skipLine(s);
  if (s==0){
    fprintf(stderr, "error, line broken when reading the range parameters\n");
    r->status=-1;
    return;
  }

  strncpy(v,s,2); r->status=atoi(v); s+=2;

  if (r->status==expectedStatus){
  } else {
    fprintf(stderr,"Error, Status=%d",r->status);
    return;
  }
  r->timestamp=parseInt(4,&s);
  s=skipLine(s);

  int i=0;

  if (remission) {
    while(s!=0){
      r->ranges[i]=parseInt(beamBytes,&s);
      if (!s)
        break;
      r->remission[i]=parseInt(3,&s);
      ++i;
    }
    i--;
  } else {
    while(s!=0){
      r->ranges[i++]=parseInt(beamBytes,&s);
    }
    i--;
  }
  r->n_ranges=i;
}




unsigned int hokuyo_readPacket(HokuyoLaser* urg, char* buf, int bufsize, int faliures){
  int i;
  int failureCount=faliures;
  if (urg->fd<=0){
    fprintf(stderr, "Invalid urg->fd\n");
    return -1;
  }

  memset(buf, 0, bufsize);
  int wasLineFeed=0;
  char* b=buf;
  while (1){
    int c=read(urg->fd, b, bufsize);
    if (! c){
      fprintf(stderr, "null" );
      usleep(25000);
      failureCount--;
    }else {
      for (i=0; i<c; i++){
	if (wasLineFeed && b[i]=='\n'){
	  b++;
	  return b-buf;
	}
	wasLineFeed=(b[i]=='\n');
      }
      b+=c;
    }
    if (failureCount<0)
      return 0;
  }
}

unsigned int hokuyo_readStatus(HokuyoLaser* urg, char* cmd){
  char buf[HOKUYO_BUFSIZE];
  int tmp = write(urg->fd,  cmd, strlen(cmd)); (void)tmp;
  while (1){
    int c=hokuyo_readPacket(urg, buf, HOKUYO_BUFSIZE,10);
    if (c>0 && !strncmp(buf,cmd+1,strlen(cmd)-1)){
      char*s=buf;
      s=skipLine(s);
      char v[3]={s[0], s[1], 0};
      return atoi(v);
    }
  }
  return 0;
    
}


#define HK_QUIT  "\nQT\n"
#define HK_SCIP  "\nSCIP2.0\n"
#define HK_BEAM  "\nBM\n"
#define HK_RESET "\nRS\n"
#define HK_SENSOR "\nPP\n"
#define wcm(cmd) write (urg->fd,  cmd, strlen(cmd))



int hokuyo_open_usb(HokuyoLaser* urg, const char* filename){
  urg->isProtocol2=0;
  urg->isInitialized=0;
  urg->isContinuous=0;
  urg->fd=open(filename, O_RDWR| O_NOCTTY | O_SYNC); //| O_NONBLOCK);

  // set terminal communication parameters
  struct termios term;
  tcgetattr(urg->fd, &term);
  term.c_cflag = CS8 | CLOCAL | CREAD; // Character size mask 8 | Ignore modem control lines | Enable receiver
  term.c_iflag = IGNPAR; // Ignore framing errors and parity errors.
  term.c_oflag = 0;
  term.c_lflag = ICANON; // Enable canonical mode
  tcflush(urg->fd, TCIFLUSH);
  tcsetattr(urg->fd, TCSANOW, &term);
  usleep(200000);

  return urg->fd;
}

/*
int hokuyo_open_serial(HokuyoLaser* urg, const char* filename, int baudrate){
  fprintf(stderr, "Baudrate: %d\n", baudrate);
  if(carmen_serial_connect(&(urg->fd),filename)<0){
    fprintf(stderr,"error\n");
    fprintf(stderr,"  carmen_serial_connect failed!\n");
    return -1;
  }
  fprintf(stderr,"connected\n");
  carmen_serial_configure(urg->fd, 19200, "8N1");
  carmen_serial_set_low_latency(urg->fd);
  fprintf(stderr,"Serial Configured\n");
  carmen_serial_ClearInputBuffer(urg->fd);
  return 1;
}
*/

int hokuyo_init(HokuyoLaser* urg, int type)
{
  int i;
  int skipping = 0;

  if (type==0){ //urg
    urg->maxBeams=URG_MAX_BEAMS;
    urg->angularResolution=URG_ANGULAR_STEP;
    urg->maxRange = URG_MAX_RANGE;
  } else if (type==1){ //utm
    urg->maxBeams=UTM_MAX_BEAMS;
    urg->angularResolution=UTM_ANGULAR_STEP;
    urg->maxRange = UTM_MAX_RANGE;
  } else if (type==2){ //ubg
    fprintf(stderr, "starting UBG\n");
    urg->maxBeams=UBG_MAX_BEAMS;
    urg->angularResolution=UBG_ANGULAR_STEP;
    urg->maxRange = UBG_MAX_RANGE;
  } else
    return -1;

  if (urg->fd<=0){
    return -1;
  }

  // stop the  device anyhow
  fprintf(stderr, "Stopping the device... "); 
  int tmp = wcm(HK_QUIT);
  tmp = wcm(HK_QUIT);
  tmp = wcm(HK_QUIT);

  urg->isContinuous=0;
  
  // put the urg in SCIP2.0 Mode
  fprintf(stderr, "Switching to enhanced mode... "); 
  int status=hokuyo_readStatus(urg, HK_SCIP);
  if (status==0){
    fprintf(stderr, "Ok\n");
    urg->isProtocol2=1;
  } else {
    fprintf(stderr, "Error. Unable to switch to SCIP2.0 Mode, please upgrade the firmware of your device\n");
    return -1;
  }

  fprintf(stderr, "Printing device information\nUSE IT TO ADJUST HARDCODED VALUES!!!");
  fprintf(stdout, "---------------------------\n");
  int wcnt = wcm(HK_SENSOR); (void) wcnt;
  char buf[HOKUYO_BUFSIZE];
  int c=hokuyo_readPacket(urg, buf, HOKUYO_BUFSIZE, 10);
  if (c > 0) {
    for (i = 0; i < c; ++i) {
      if (buf[i] == ';')
        skipping = 1;
      else if (buf[i] == '\n')
        skipping = 0;
      if (! skipping)
        fprintf(stdout, "%c", buf[i]);
    }
    fflush(stdout);
  }
  fprintf(stdout, "---------------------------\n");

  fprintf(stderr, "Device initialized successfully\n");
  urg->isInitialized=1;
  return 1;
}

int hokuyo_startContinuous(HokuyoLaser* urg, int startStep, int endStep, int clusterCount, int remission){
  if (! urg->isInitialized)
    return -1;
  if (urg->isContinuous)
    return -1;

  // switch on the laser
  fprintf(stderr, "Switching on the laser emitter...  "); 
  int status=hokuyo_readStatus(urg, HK_BEAM);
  if (! status){
    fprintf(stderr, "Ok\n"); 
  } else {
    fprintf(stderr, "Error. Unable to control the laser, status is %d\n", status);
    return -1;
  }

  urg->startBeam = startStep;
  urg->endBeam   = endStep;
  urg->remission = remission;

  char command[1024];
  if (!remission) {
    sprintf (command, "\nMD%04d%04d%02d000\n", startStep, endStep, clusterCount);
  } else {
    sprintf (command, "\nME%04d%04d%02d000\n", startStep, endStep, clusterCount);
  }

  status=hokuyo_readStatus(urg, command);
  if (status==99 || status==0){
    fprintf(stderr, "Continuous mode started with command %s\n", command);
    urg->isContinuous=1;
    return 1;
  }
  fprintf(stderr, "Error. Unable to set the continuous mode, status=%02d\n", status);

  return -1;
}

int hokuyo_stopContinuous(HokuyoLaser* urg){
  if (! urg->isInitialized)
    return -1;
  if (! urg->isContinuous)
    return -1;

  int status=hokuyo_readStatus(urg, HK_QUIT);
  if (status==0){
    fprintf(stderr, "Ok\n");
    urg->isContinuous=0;
  } else {
    fprintf(stderr, "Error. Unable to stop the laser\n");
    return -1;
  }
  return 1;
}

int hokuyo_reset(HokuyoLaser* urg){
  if (! urg->isInitialized)
    return -1;

  int status=hokuyo_readStatus(urg, HK_RESET);
  if (status==0){
    fprintf(stderr, "Ok\n");
    urg->isContinuous=0;
  } else {
    fprintf(stderr, "Error. Unable to reset laser\n");
    return -1;
  }
  return 1;
}

int hokuyo_close(HokuyoLaser* urg){
  if (! urg->isInitialized)
    return -1;
  hokuyo_stopContinuous(urg);
  close(urg->fd);
  urg->isProtocol2=0;
  urg->isInitialized=0;
  urg->isContinuous=0;
  urg->fd=-1;
  return 1;
}
