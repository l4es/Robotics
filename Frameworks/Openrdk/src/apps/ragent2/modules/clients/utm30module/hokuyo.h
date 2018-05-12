#ifndef HOKUYO_H
#define HOKUYO_H

#define HOKUYO_BUFSIZE 16384
//#define HOKUYO_BUFSIZE 32768
#define HOKUYO_MAX_BEAMS 2048

#define URG_ANGULAR_STEP (M_PI/512.)
#define URG_MAX_BEAMS 768
#define URG_MAX_RANGE (5600) // 5.6m with the new protocol

#define UTM_ANGULAR_STEP (M_PI/720.)
#define UTM_MAX_BEAMS    1080
#define UTM_MAX_RANGE    (30000) // 30.0 m

#define UBG_ANGULAR_STEP (M_PI/512.)
#define UBG_MAX_BEAMS	768
#define UBG_MAX_RANGE	4095

typedef struct HokuyoRangeReading{
  int timestamp;
  int status;
  int n_ranges;
  unsigned short ranges[HOKUYO_MAX_BEAMS];
  unsigned short remission[HOKUYO_MAX_BEAMS];
  unsigned short startStep, endStep, clusterCount;
} HokuyoRangeReading;

typedef struct HokuyoLaser{
  int fd;
  int baudrate;
  int isProtocol2;
  int isContinuous;
  int isInitialized;
  int maxBeams;
  double angularResolution;
  unsigned short maxRange; /// maxRange in mm
  int startBeam;
  int endBeam;
  int remission;
} HokuyoLaser;

// opens the urg, returns <=0 on failure
int hokuyo_open_usb(HokuyoLaser* urg, const char* filename);

// opens the urg, returns <=0 on failure
int hokuyo_open_serial(HokuyoLaser* urg, const char* filename, int baudrate);

// type 0: urg
// type 1: utm
// type 2: ubg
// initializes the urg and sets it to the new scip2.0 protocol
// returns <=0 on failure
int hokuyo_init(HokuyoLaser* urg, int type);

// reads a packet into the buffer
unsigned int hokuyo_readPacket(HokuyoLaser* urg, char* buf, int bufsize, int faliures);

// starts the continuous mode
int hokuyo_startContinuous(HokuyoLaser* urg, int startStep, int endStep, int clusterCount, int remission);

// stops the continuous mode
int hokuyo_stopContinuous(HokuyoLaser* urg);

int hokuyo_reset(HokuyoLaser* urg);
int hokuyo_close(HokuyoLaser* urg);
void hokuyo_parseReading(HokuyoRangeReading* r, char* buffer, int remission);

#endif

