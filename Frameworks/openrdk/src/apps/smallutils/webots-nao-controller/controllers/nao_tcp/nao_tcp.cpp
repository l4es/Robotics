#include <iostream>
#include <vector>
#include <iterator>
#include <fcntl.h>

#include "../../../../ragent2/modules/clients/webotsclientmodule/webots.h"
#include <math.h>
#include <string.h>
#include <device/robot.h>
#include <device/servo.h>
#include <device/camera.h>
#include <device/distance_sensor.h>
#include <device/accelerometer.h>
#include <device/touch_sensor.h>
#include <device/led.h>
#include <device/emitter.h>
#include <device/receiver.h>
#include <device/gps.h>
#include <stdio.h>

#define TIME_STEP 40

DeviceTag jointDevice[NAO_JOINTS_NUMBER],
          camera,left_ultrasound_sensor,right_ultrasound_sensor,accelerometer,
          fsrDevice[8],
          emitter,receiver,logo_led,gps;

int color;
int number = 0;
std::vector<int> sock;
int serversock = 0, commandsock = 0;
bool standingForCommander;
float jointValue[NAO_JOINTS_NUMBER] = INITIAL_POSE;	

int getServerSocket(int port);
int getClientSocket(int serversock);

void * acceptClientThread(void *) {
	int serversock = getServerSocket(WEBOTSPORT + 50 * color + number);
  if (serversock == -1)
			std::cout << "Robot "<<number<<" : Error in server socket creation..." << std::endl;
	while (true) {
		int clientsock = getClientSocket(serversock);
		if (clientsock == -1)
			std::cout << "Robot "<<number<<" : Error in connection..." << std::endl;
		else {
			robot_console_printf("Robot %d : New client accepted\n",number);
			sock.push_back(clientsock);
		}
	}
	return 0;
}

void reset(void) {
	const char *name = robot_get_name();
	     if (strcmp(name,"red goal keeper")==0)  { number=1; color=0; robot_console_printf(" %d %d \n",number,color);}
	else if (strcmp(name,"blue goal keeper")==0) { number=1; color=1; robot_console_printf(" %d %d \n",number,color);}
	else if (strcmp(name,"red player 1")==0)     { number=2; color=0; robot_console_printf(" %d %d \n",number,color);}
	else if (strcmp(name,"blue player 1")==0)    { number=2; color=1; robot_console_printf(" %d %d \n",number,color);}
	else if (strcmp(name,"red player 2")==0)     { number=3; color=0; robot_console_printf(" %d %d \n",number,color);}
	else if (strcmp(name,"blue player 2")==0)    { number=3; color=1; robot_console_printf(" %d %d \n",number,color);}
	else if (strcmp(name,"red player 3")==0)     { number=4; color=0; robot_console_printf(" %d %d \n",number,color);}
	else if (strcmp(name,"blue player 3")==0)    { number=4; color=1; robot_console_printf(" %d %d \n",number,color);}
	
	jointDevice[0]	= robot_get_device("HeadYaw");
	jointDevice[1]	= robot_get_device("HeadPitch");

	jointDevice[2]	= robot_get_device("LShoulderPitch");
	jointDevice[3]	= robot_get_device("LShoulderRoll");
	jointDevice[4]	= robot_get_device("LElbowYaw");
	jointDevice[5]	= robot_get_device("LElbowRoll");

	jointDevice[6]	= robot_get_device("LHipYawPitch");
	jointDevice[7]	= robot_get_device("LHipRoll");
	jointDevice[8]	= robot_get_device("LHipPitch");
	jointDevice[9]	= robot_get_device("LKneePitch");
	jointDevice[10]	= robot_get_device("LAnklePitch");
	jointDevice[11]	= robot_get_device("LAnkleRoll");

	jointDevice[12]	= robot_get_device("RHipYawPitch");
	jointDevice[13]	= robot_get_device("RHipRoll");
	jointDevice[14]	= robot_get_device("RHipPitch");
	jointDevice[15]	= robot_get_device("RKneePitch");
	jointDevice[16]	= robot_get_device("RAnklePitch");
	jointDevice[17]	= robot_get_device("RAnkleRoll");

	jointDevice[18]	= robot_get_device("RShoulderPitch");
	jointDevice[19]	= robot_get_device("RShoulderRoll");
	jointDevice[20]	= robot_get_device("RElbowYaw");
	jointDevice[21]	= robot_get_device("RElbowRoll");

	for (int i = 0; i < 22; i++)
		servo_enable_position(jointDevice[i],TIME_STEP);
	gps= robot_get_device("gps");
	gps_enable(gps,20);

	camera=robot_get_device("camera");
	camera_enable(camera,4*TIME_STEP);

	left_ultrasound_sensor = robot_get_device("left ultrasound sensor");
	distance_sensor_enable(left_ultrasound_sensor,TIME_STEP);

	right_ultrasound_sensor = robot_get_device("right ultrasound sensor");
	distance_sensor_enable(right_ultrasound_sensor,TIME_STEP);

	fsrDevice[0] = robot_get_device("LFsrFL");
	fsrDevice[1] = robot_get_device("LFsrFR");
	fsrDevice[2] = robot_get_device("LFsrBL");
	fsrDevice[3] = robot_get_device("LFsrBR");
	fsrDevice[4] = robot_get_device("RFsrFL");
	fsrDevice[5] = robot_get_device("RFsrFR");
	fsrDevice[6] = robot_get_device("RFsrBL");
	fsrDevice[7] = robot_get_device("RFsrBR");
	
	for(int i=0;i<8;i++)
		touch_sensor_enable(fsrDevice[i],TIME_STEP);

	serversock = getServerSocket(WEBOTSPORT + 50 * color + number + 100);
	standingForCommander = false;
	commandsock = 0;

	pthread_t pthr;
	pthread_create(&pthr,NULL,acceptClientThread,0);
}

void * acceptCommanderThread(void *) {
	while (commandsock <= 0) {
		robot_console_printf("Waiting for a commander connection...\n");
		commandsock = getClientSocket(serversock);
		robot_console_printf("Commander connection accepted!\n");
	}
	return 0;
}

void standForCommander() {
	if (!standingForCommander) {
		robot_console_printf("NOT standing --> STANDING \n");
		standingForCommander = true;
		pthread_t pthr;
		pthread_create(&pthr,NULL,acceptCommanderThread,0);
	}
	usleep(100);
}

int run (int ms) {

	if (commandsock) {
		if (standingForCommander) {
			robot_console_printf("STANDING --> NOT standing \n");
			standingForCommander = false;
		}
	}
	else {
		standForCommander();
		return TIMESTEP;
	}
	
	WebotsData wdata;

	wdata.timestamp = robot_get_time();
	
	const float* g_matrix = gps_get_matrix(gps);
	float eulerAngles[3];
	gps_euler(g_matrix,eulerAngles);
	wdata.gps[0] = gps_position_x(g_matrix);
	wdata.gps[1] = gps_position_y(g_matrix);
	wdata.gps[2] = gps_position_z(g_matrix);
	
	wdata.gps[3] = eulerAngles[0];//inclinometro rispetto a x
	wdata.gps[4] = eulerAngles[1];//angolo di compass rispetto a y
	wdata.gps[5] = eulerAngles[2];//inclinometro rispetto a z

	// LI: così funziona meglio
	float theta;
	theta=atan2(g_matrix[8],g_matrix[0]);
	wdata.gps[4] = theta;
  
	//robot_console_printf("EulerX %f : Commander accepted\n",wdata.gps[3]);
	//robot_console_printf("EulerY %f / %f\n",wdata.gps[4],theta);
	//robot_console_printf("EulerZ %f : Commander accepted\n",wdata.gps[5]);

	for (int i = 0; i < NAO_JOINTS_NUMBER; i++) {
		wdata.joints[i] = servo_get_position(jointDevice[i]);
		// printf("%f\n",wdata.joints[i]);
	}

	for (int i = 0; i < NAO_FSR_NUMBER * NAO_FSR_DIMENSION; i++)
		wdata.fsr[i] = touch_sensor_get_value(fsrDevice[i]);
	
	wdata.sonarRight = distance_sensor_get_value(right_ultrasound_sensor);
	wdata.sonarLeft = distance_sensor_get_value(left_ultrasound_sensor);
	memcpy(wdata.image,camera_get_image(camera),sizeof(wdata.image));
		
	// SEND SENSORS VALUES TO CLIENTS
	std::vector<int>::iterator itSock = sock.begin();
	while (itSock != sock.end()) {
		bool broken = false;
		int current = * itSock;
		unsigned char * pointer = (unsigned char*) &wdata;
		uint sent = 0;
		while (sent < sizeof(wdata)) {
			int i = write(current, pointer, sizeof(wdata) - sent);
			if (i > 0) {
				sent += i;
				pointer += i;
			}
			else {
				broken= true;
				break;
			}
		}
		if (broken) {
			itSock = sock.erase(itSock);
			robot_console_printf("Robot %d: Client disconnetted\n",number);
		}
		else
			itSock++;
	}
	// RECEIVE COMMAND FROM RDK
	bool broken = false;
	uint received = 0;
	unsigned char * pointer = (unsigned char *) &jointValue;
	while (received < sizeof(jointValue)) {
		int i = read(commandsock, pointer, sizeof(jointValue) - received);
		if (i > 0) {
			received += i;
			pointer += i;
		}
		else {
			broken= true;
			break;
		}
	}
	if (!broken) {
		for (size_t i = 0; i < NAO_JOINTS_NUMBER; i++)
			servo_set_position(jointDevice[i], jointValue[i]);
	}
	else {
		close(commandsock);
		robot_console_printf("Robot %d : Commander disconneted\n",number);
		commandsock = 0;
	}
	return TIMESTEP;
}

int main (void) {
	robot_live(reset);
	robot_run(run);
	return 0;
}


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>         /* definition of struct sockaddr_in */
#include <netdb.h>              /* definition of gethostbyname */
#include <arpa/inet.h>          /* definition of inet_ntoa */
#include <unistd.h>             /* definition of close */

int getServerSocket(int port) {
    struct sockaddr_in address;

    /* create the socket */
    int ssock = socket(AF_INET, SOCK_STREAM, 0);
    if (ssock == -1) {
        std::cout << "Robot "<<number<<" cannot create server socket" << std::endl;
        return -1;
    }

    /* fill in socket address */
    memset(&address, 0, sizeof(struct sockaddr_in));
    address.sin_family = AF_INET;
    address.sin_port = htons((unsigned short) port);
    address.sin_addr.s_addr = INADDR_ANY;

    /* bind to port */
    if (bind(ssock,(struct sockaddr *) &address, sizeof(struct sockaddr))==-1) {
        std::cout << "Robot "<<number<<" cannot bind port " << port << std::endl;
        return -1;
    }

    /* listen for connections */
    if (listen(ssock, 10) == -1) {
        std::cout << "Robot "<<number<<" cannot listen for connections on port " << port << std::endl;
        return -1;
    }
    robot_console_printf("Robot %d: Waiting for connections on port %d\n",number,port);

    return ssock;
}

int getClientSocket(int serversock) {
	int csock;
	struct sockaddr_in client;
	unsigned int asize;
	struct hostent *client_info;
	
	asize = sizeof(struct sockaddr_in);
	csock = accept(serversock, (struct sockaddr *) &client, &asize);
	if (csock == -1) {
		std::cout << "Robot " << number << "cannot accepts client" << std::endl;
		return -1;
	}
	else {
		client_info = gethostbyname((char *) inet_ntoa(client.sin_addr));
		robot_console_printf("Robot %d: Connection from %s\n",number,client_info->h_name);
	}

	return csock;
}

