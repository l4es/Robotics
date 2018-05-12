
#ifndef  SHAREDMEMORY_TRAITS_INC
#define  SHAREDMEMORY_TRAITS_INC

//#include <nao/object/naojoints.h>
#include <string>

//static const std::string MOTION_SHM_NAME = "MotionSharedMemory";
//static const std::string IMAGE_SHM_NAME = "ImageSharedMemory";

//static const size_t JOINTS_VALUES_SIZE = Nao::NaoJoints::NAO_JOINTS_COUNT;
//static const size_t FORCE_SENSOR_SIZE  = 8;
//static const size_t CAMERAMATRIX_SIZE  = 12; // R=3x3 T=3x1 -> M=3x4

#define MEM_KEY 123

#define TRAFFIC_RDK_NAOQI_KEY 459
#define TRAFFIC_NAOQI_RDK_KEY 789

#ifndef SHM_R
#define SHM_R 0400
#endif
#ifndef SHM_W
#define SHM_W 0660
#endif

#define RDK 0
#define NAOQI 1

#endif   /* ----- #ifndef SHAREDMEMORY_TRAITS_INC  ----- */
