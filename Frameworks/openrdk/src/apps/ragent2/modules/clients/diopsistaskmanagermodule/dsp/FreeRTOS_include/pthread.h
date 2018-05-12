/**
 * \addtogroup posix POSIX
 * @{
 * 
 * Implementation of POSIX API using FreeRTOS API
 */

/**
 * \defgroup pthread POSIX Thread Call
 * @{
 * 
 * Implementation of POSIX Threads, interface between posix thread, mutex and freertos task, semaphore
 */

/**
 * \file
 * POSIX Thread
 * \author Dennis Tsang <dennis@amonics.com>
 */

/**
 * \page thread POSIX THREAD MAPPING
 * This header file contains the mixed operation for FreeRTOS Task and CRTHREAD, the selection
 * of which operation is determined by \#define directives
 * 
 * \section sec1 INTRODUCTION
 * \par Principle of FreeRTOS Scheduler
 * 
 * When a task is created, the required dynamic memory (minimal stack size) will be acquired from heap.
 * 
 * The created task will be executed in a round-robin fashion, i.e Task A -> Task B -> Idle Task
 * -> Task A -> Task B -> ... 
 * 
 * During the context switch between differnt tasks (except Idle Task), all the contents in 
 * the register stack will be saved, and the task will resume correctly when the saved data
 * are loaded back to the stack.
 * 
 * \par Principle of CRTHREAD Scheduler
 * 
 * When the application task creates a CRTHREAD, the coroutine function pointer is added to 
 * the coroutine queue if the queue is not full, otherwise, the CRTHEAD is skipped.
 * 
 * The created CTHREADs will be executed during the execution of the Idle task.
 * 
 * If another instance of the same corountine function is running (active), the added newly 
 * created CRTHREAD will become inactive until the all the previous instances have been completed.  
 * 
 * NOTE:
 * A problem may occur when many CRTHREADs are created so that the the queue is full at most time.
 * In such case, most of the CRTHREADs will be skipped. Whenever one of the CRTHREAD is completed
 * and a space is available in the queue, only one new CRTHREAD (most probably, the one after usleep())
 * can be added. 
 * 
 * To prevent this to occur, the application task should check the return value of
 * pthread_create(). If it is -1, it indicate the queue is full and the application should handle 
 * the issue. Alternatively, you should increase the size of queue in define.h
 * 
 *  
 * \par Rationale for pthread_join:
 * \verbatim
   CRTHREAD QUEUE
   ==============
   crlist[0] = {88,  disable,        arg0}
   crlist[1] = {0,   CRTHREAD_EMPTY, arg1}
   crlist[2] = {124, 0,              arg3}
   crlist[3] = {125, 0,              arg4}
   ...
   crlist[MAX_CRTHREAD-1] = {0, CRTHREAD_EMPTY, argn}
  
   Implementation at pthread_create()
   ==================================
   Thread ID = an integer greater than 0;
    
   Implementation at pthread_join()
   ==================================
   Finish condition occurs when the thread id does not exist in the list as a result of
   removal by coroutine scheduler  
  
   Problem arised if using address of crthread as id:
  
                 disable finishes              
              within 1 round and set 
         crthread[5] = CRTHREAD_EMPTY        Task B will
                Task B should wake        continue to sleep
                                |               |
                            [2] |           [4] |
                               \|/             \|/
          +--------+--------+------+--------+--------+------+
          | Task A | Task B | Idle | Task A | Task B | Idle |
          +--------+--------+------+--------+--------+------+
                       |      /|\       |               /|\
                   [1] |       |    [3] |                |
                       +-------+        +----------------+
    Task B call pthread_create()     Task A call pthread_create()
    crthread[5] = disable            crthread[5] = xxx
    thread_idB = &crthread[5]        thread_idA = &crthread[5]
    Task B will sleep
   
   EVENTS:
   [1] Task B calls pthread_create(), one of the crthread, say crthread[5], is assigned to
       the target function (e.g. crthread[5] = disable, hence thread_idB = &crthread[5]).
       Task B will sleep immediately, and expect to wake until disable() is completed.
   [2] Idle task will execute the CR_SCHEDULER. If disable() is completed upon exiting the 
       scheduler (i.e. no sleep is called), then crthread[5] = CRTHREAD_EMPTY. Indicating
       Task B should wake up.
   [3] If Task A now create a thread and takes up the same position, (i.e. crthread[5] = xxx)
       The CRTHREAD_EMPTY status will be erased.
   [4] When Task B checks for the content in thread_idB, it is not equal to CRTHREAD_EMPTY,
       and it continue to sleep.
   \endverbatim
 *
 * \section sec2 COMPILE TIME CONFIGURATION
 * \par Set the following in <.config_freertos_posix> before compiling
 * \verbatim
    FREERTOS_SCHED                    =   y
    CRTHREAD_SCHED                    =   y
   \endverbatim
 */ 

#ifndef PTHREAD_H_
#define PTHREAD_H_  1

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/** pthread_attr_t -> unsigned char (expandable if more attributes are needed) */
#define pthread_attr_t      unsigned char  

/**
 * \brief MARCO for int pthread_attr_init(pthread_attr_t *attr)
 * \details The pthread_attr_init() function shall initialize a thread attributes object 
 *        	attr with the default value for all of the individual attributes used by a 
 *        	given implementation.
 * \param attr_ptr pointer to attr variable
 * \return Upon successful completion, pthread_attr_init() shall return a value of 0; 
 *         otherwise, an error number shall be returned to indicate the error. (not implemented)
 */
#define pthread_attr_init(attr_ptr)        while(0)


enum
{
  PTHREAD_SCOPE_SYSTEM,
  /** attribute ID (for coroutine Thread) */
  #define PTHREAD_SCOPE_SYSTEM    PTHREAD_SCOPE_SYSTEM
  PTHREAD_SCOPE_PROCESS
  /** attribute ID (for freeRTOS Thread) */
  #define PTHREAD_SCOPE_PROCESS   PTHREAD_SCOPE_PROCESS
};

/**
 * \brief MARCO for int pthread_attr_setscope(pthread_attr_t *attr, int contentionscope)
 * \details pthread_attr_setscope() functions, respectively, shall set the contentionscope 
 *          attribute in the attr object.
 * \param attr_ptr pointer to attr variable
 * \param attr_id attribute id (e.g. PTHREAD_SCOPE_SYSTEM, PTHREAD_SCOPE_PROCESS)
 * \return If successful, the pthread_attr_setscope() functions shall return zero; 
 *         otherwise, an error number shall be returned to indicate the error. (not implemented)
 */
#define pthread_attr_setscope(attr_ptr, attr_id)    *attr_ptr = attr_id


/** pthread_t -> xTaskHandle (void *, and tskTCB, refer to task.h, and task.c) */
#define pthread_t           xTaskHandle

/**
 * \brief Create a new thread, with attributes specified by attr. 
 *          Upon successful completion, pthread_create() shall store the ID of 
 *          the created thread in the location referenced by thread.
 *          The thread is created executing start_routine with arg as its sole argument.
 * \param thread handler for task
 * \n            if caller function is a coroutine, content of thread should be declared static or global
 * \param attr attribute pointer related to the thread (dynamic selection of scheduling policy)
 * \n           NULL use FreeRTOS Task
 * \n           PTHREAD_SCOPE_SYSTEM use crthread
 * \param start_routine function pointer for the task
 * \param arg arguments to passed to task
 * \n         for crthread, content of arg should be declared static or global
 * 
 * \return If successful, the pthread_create() function shall return zero; 
 * \n      Otherwise, an error number (-1) shall be returned to indicate the error.
 * \remarks The task is created with minimal stack size and idle priority 
 */
extern int pthread_create(pthread_t* thread, pthread_attr_t* attr, void* (*start_routine)(void*), void* arg);


/**
 * \brief MARCO for int pthread_join(pthread_t thread, void **value_ptr);
 * \details wait for thread termination 
 * \param thread handler for target thread
 * \param value_ptr resources pointed by the pointer will be released (not implemented yet)
 * \return: (not implemented)
 * \n       Upon sucess, return 0
 * \n       Upon failure, -1, errno raised
 * \n       ESRCH:   no such thread
 * \n       EDEADLK: detected deadlock
 * \n       EINVAL:  target thread is not joinable
 * \remarks 
 * Limitations: 
 * \li do not work for (Caller, Target) = (FreeRTOS Task, FreeRTOS Task)
 * \li work for (Caller, Target) = (FreeRTOS Task, coroutine_st)
 * \li work for (Caller, Target) = (coroutine_st, coroutine_st)
 */
#ifdef CRTHREAD_SCHED
#define pthread_join(thread, value_ptr)     while(1){\
                                                if(thread==(pthread_t)0) break;\
                                                char pthread_join_i;\
                                                char pthread_join_index = -1;\
                                                for(pthread_join_i=0; pthread_join_i<MAX_CRTHREAD; pthread_join_i++){\
                                                    if(thread == crlist[pthread_join_i].id){\
                                                        pthread_join_index = pthread_join_i;\
                                                    }\
                                                }\
                                                if(pthread_join_index < 0) break;\
                                                else usleep(0);\
                                             }
#else /* NOT CRTHREAD_SCHED */
#define pthread_join(thread, value_ptr)     while(0)
#endif /* NOT CRTHREAD_SCHED */



/** pthread_mutex_t -> xSemaphoreHandle (xQueueHandle, void *, xQUEUE* refer to semphr.h, queue.h, queue.c) */
#define pthread_mutex_t     xSemaphoreHandle
/** pthread_mutexattr_t -> int */
#define pthread_mutexattr_t int

/**
 * \brief The pthread_mutex_init() function shall initialize the mutex referenced 
 *        by mutex with attributes specified by attr. If attr is NULL, the default 
 *        mutex attributes are used; the effect shall be the same as passing the 
 *        address of a default mutex attributes object. Upon successful initialization, 
 *        the state of the mutex becomes initialized and unlocked.
 * \param mutex Handle to the created semaphore.
 * \param attr not implemented, use NULL
 * \return If successful, pthread_mutex_init() functions shall return zero; 
 * \n      Otherwise, an error number shall be returned to indicate the error.
 */
extern int pthread_mutex_init(pthread_mutex_t *mutex, pthread_mutexattr_t *attr);

/**
 * \brief The mutex object referenced by mutex shall be locked by calling 
 *        pthread_mutex_lock(). If the mutex is already locked, the calling thread 
 *        shall block until the mutex becomes available. This operation shall return 
 *        with the mutex object referenced by mutex in the locked state with the calling
 *        thread as its owner.
 * \param mutex Handle to the created semaphore.
 * \return If successful, the pthread_mutex_lock() functions shall return zero; 
 * \n      Otherwise, an error number shall be returned to indicate the error.
 */         
extern int pthread_mutex_lock(pthread_mutex_t *mutex);

/** 
 * \brief The pthread_mutex_unlock() function shall release the mutex object referenced 
 *        by mutex. The manner in which a mutex is released is dependent upon the mutex's 
 *        type attribute. If there are threads blocked on the mutex object referenced by
 *        mutex when pthread_mutex_unlock() is called, resulting in the mutex becoming 
 *        available, the scheduling policy shall determine which thread shall acquire 
 *        the mutex.
 * 
 * \param mutex Handle to the created semaphore.
 * \return If successful, pthread_mutex_unlock() functions shall return zero; 
 * \n      Otherwise, an error number shall be returned to indicate the error.
 * 
 * \remarks  
 * pthread_mutex_unlock() -> xSemaphoreGive()
 * \li This must not be used from an ISR. See xSemaphoreGiveFromISR() for an alternative which
 * can be used from an ISR.
 ********************************************************************************************/
extern int pthread_mutex_unlock(pthread_mutex_t *mutex);


/*
 * CRTHREAD
 */
#ifdef CRTHREAD_SCHED

/** coroutine thread should take the form: void* foo(void* arg) */
typedef void* (*crthread_t)(void* arg);

/** indicate that no crthread has been scheduled */
#define CRTHREAD_EMPTY  (((crthread_t) 0 ) + MAX_CRTHREAD)

/** crountine thread */
struct crElement_t
{
  /** crthread ID; 0  = no task; >0 = valid id */ 
  pthread_t id;
  /** function pointer type for coroutine */
  crthread_t crthread;
  /** pointer to arguments assocated with the crthread */
  void* arg;
};

/** Array of crElements. Each element contains a function pointer to a crthread and a crthread_id */
extern struct crElement_t crlist[];

extern pthread_t crthread_id_counter;
/**
 * \brief MARCO for pthread_t next_crthread_id(void)
 * \details return the next valid crthread id
 * \return a unique and valid crthread id > 0
 */
#define next_crthread_id()      (crthread_id_counter++); \
                                 if(crthread_id_counter == (pthread_t)0) crthread_id_counter++
#endif /* CRTHREAD_SCHED */

/**
 * \brief System Thread (currently corountine scheduler)
 * \param ptr (not used)
 * \remarks main program should create this thread if coroutine scheduler is needed 
 */
extern void* sys_thread(void* ptr);

/** 
 * \example pthread_ex.c
 * This is an example of how to use the posix thread module.
 */

#endif /* PTHREAD_H_  */

/** @} */
/** @} */
