// it is better to declare some define to be used as property names, in order to avoid misspelling in strings,
// declare here those defines for all your properties and use these; for example:

// #define PROPERTY_MY_BEAUTIFUL_STRING "myBeautifulString"
// #define PROPERTY_MAX_SPEED "maxSpeed"
// ...
#define PROPERTY_CMD_START "cmds/start"
#define PROPERTY_STATUS "status"

#define TASK_NAME "name"
#define TASK_ADDRESS "address"
#define TASK_PRIORITY "priority"
#define TASK_VARIABLE "variable"

#define PROPERTY_PARAMS "params"
#define TASK1_BASENAME PROPERTY_PARAMS"/task1"
#define PROPERTY_PARAMS_TASK1_NAME     TASK1_BASENAME"/"TASK_NAME
#define PROPERTY_PARAMS_TASK1_ADDRESS  TASK1_BASENAME"/"TASK_ADDRESS
#define PROPERTY_PARAMS_TASK1_PRIORITY TASK1_BASENAME"/"TASK_PRIORITY
#define PROPERTY_PARAMS_TASK1_VARIABLE TASK1_BASENAME"/"TASK_VARIABLE

#define TASK2_BASENAME PROPERTY_PARAMS"/task2"
#define PROPERTY_PARAMS_TASK2_NAME     TASK2_BASENAME"/"TASK_NAME
#define PROPERTY_PARAMS_TASK2_ADDRESS  TASK2_BASENAME"/"TASK_ADDRESS
#define PROPERTY_PARAMS_TASK2_PRIORITY TASK2_BASENAME"/"TASK_PRIORITY
#define PROPERTY_PARAMS_TASK2_VARIABLE TASK2_BASENAME"/"TASK_VARIABLE

static const std::string DTM_IDLE = "idle";
static const std::string DTM_START = "starting task manager";
static const std::string DTM_RUNNING = "task manager is running";
static const std::string DTM_SND = "sending a msg";
static const std::string DTM_RCV = "receiving a msg";
static const std::string DTM_ALLOC_MEMORY = "allocating memory";
static const std::string DTM_ALLOC_MEMORY_ERROR = "error allocating memory";
static const std::string DTM_TASK_START = "starting task";
static const std::string DTM_TASK_START_ERROR = "error starting task";

