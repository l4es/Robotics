#ifndef __WEBOTS_H
#define __WEBOTS_H

#define WEBOTSHOST "127.0.0.1"
#define WEBOTSPORT 51510
#define TIMESTEP 40

#define POSX 0
#define POSZ 1
#define POSY 2

#define NAO_JOINTS_NUMBER 22
#define NAO_FSR_NUMBER 2
#define NAO_FSR_DIMENSION 4
#define NAO_INERTIAL_DIMENSION 3
#define NAO_GPS_DIMENSION 6
#define WEBOTS_CAMERA_W 160
#define WEBOTS_CAMERA_H 120

#define INITIAL_POSE {0.00, 0.00, 1.40, 0.35, -1.40, -1.57, 0.00, -0.44, 0.00, 0.70, -0.35, 0.00, 0.00, -0.44, 0.00, 0.70, -0.35, 0.00, 1.40, -0.35, 1.40, 1.57}

enum EwebotsColor {webotsRED, webotsBLUE};

struct WebotsData {
	float joints[NAO_JOINTS_NUMBER];
	float fsr[NAO_FSR_NUMBER * NAO_FSR_DIMENSION];
	float inertial[NAO_INERTIAL_DIMENSION];
	float sonarRight;
	float sonarLeft;
	float gps[NAO_GPS_DIMENSION];
	unsigned char image[WEBOTS_CAMERA_W * WEBOTS_CAMERA_H * 3];
	double timestamp;
};

struct WebotsSupervisorData {
  float ballPose[3];
};

#endif
