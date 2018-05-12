#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "FreeRTOS_include/queue.h"
#include "DBIOS/DBIOS_macro.h"
#include "DSPAbstract.h"
#include "DSPAbstract_ResourceManager.h"

/* Constants used with the cRxLock and cTxLock structure members. */
#define queueUNLOCKED	( ( signed portBASE_TYPE ) -1 )
#define queueERRONEOUS_UNBLOCK					( -1 )
/* Effectively make a union out of the xQUEUE structure. */
#define pxMutexHolder				pcTail
#define uxQueueType					pcHead
#define queueQUEUE_IS_MUTEX			NULL

#ifdef __cplusplus
extern "C" {
#endif


/*-----------------------------------------------------------
 * PUBLIC LIST API documented in list.h
 *----------------------------------------------------------*/

DA_long *_M_memcpy(DA_long *dest, DA_long *src, DA_int n){
	DA_int i;
	for (i=0; i<n; i++){
		*dest++ = *src++;
	}
	return dest;
}

/*
 * Macro to mark a queue as locked.  Locking a queue prevents an ISR from
 * accessing the queue event lists.
 */
#define prvLockQueue( pxQueue )			\
{						\
		++( _M_STRUCTI(xQUEUE, pxQueue)->xRxLock );		\
		++( _M_STRUCTI(xQUEUE, pxQueue)->xTxLock );		\
}

portTickType xTaskGetTickCount(void){
		struct timeval tv;
		gettimeofday(&tv, NULL);
		return tv.tv_sec*1000000 + tv.tv_usec;
}

static void prvCopyDataToQueue( xQUEUE *pxQueue, const void *pvItemToQueue, portBASE_TYPE xPosition )
{
	printf("cpq-- %x\n",(_M_STRUCTI(xQUEUE, pxQueue)->pcWriteTo));
	if( _M_STRUCTI(xQUEUE, pxQueue)->uxItemSize == 0 )
	{
	}
	else if( xPosition == queueSEND_TO_BACK )
	{
		printf("cpq2-- %ld\n",*((DA_long *)pvItemToQueue+1));
		_M_memcpy(
				(DA_long *)((_DBIOS_DM_I_mmapped_base + (DA_long)(_M_STRUCTI(xQUEUE, pxQueue)->pcWriteTo))),
				(DA_long *)pvItemToQueue,
				( unsigned ) _M_STRUCTI(xQUEUE, reinterpret_cast<long>(pxQueue))->uxItemSize
				);
		_M_STRUCTI(xQUEUE, pxQueue)->pcWriteTo += _M_STRUCTI(xQUEUE, pxQueue)->uxItemSize;
		if( _M_STRUCTI(xQUEUE, pxQueue)->pcWriteTo >= _M_STRUCTI(xQUEUE, pxQueue)->pcTail )
		{
			_M_STRUCTI(xQUEUE, pxQueue)->pcWriteTo = _M_STRUCTI(xQUEUE, pxQueue)->pcHead;
		}
	}
	else
	{
		_M_memcpy(
				(DA_long *)((_DBIOS_DM_I_mmapped_base + (DA_long)(_M_STRUCTI(xQUEUE, pxQueue)->pcReadFrom))),
				(DA_long *)pvItemToQueue,
				( unsigned ) _M_STRUCTI(xQUEUE, pxQueue)->uxItemSize
				);
		_M_STRUCTI(xQUEUE, pxQueue)->pcReadFrom -= _M_STRUCTI(xQUEUE, pxQueue)->uxItemSize;
		if( _M_STRUCTI(xQUEUE, pxQueue)->pcReadFrom < _M_STRUCTI(xQUEUE, pxQueue)->pcHead )
		{
			_M_STRUCTI(xQUEUE, pxQueue)->pcReadFrom = ( _M_STRUCTI(xQUEUE, pxQueue)->pcTail - _M_STRUCTI(xQUEUE, pxQueue)->uxItemSize );
		}		
	}

	++( _M_STRUCTI(xQUEUE, pxQueue)->uxMessagesWaiting );
}
/*-----------------------------------------------------------*/

static void prvCopyDataFromQueue( xQUEUE * const pxQueue, const void *pvBuffer )
{
	if( _M_STRUCTI(xQUEUE, pxQueue)->uxQueueType != queueQUEUE_IS_MUTEX )
	{
		_M_STRUCTI(xQUEUE, pxQueue)->pcReadFrom += _M_STRUCTI(xQUEUE, pxQueue)->uxItemSize;
		if( _M_STRUCTI(xQUEUE, pxQueue)->pcReadFrom >= _M_STRUCTI(xQUEUE, pxQueue)->pcTail )
		{
			_M_STRUCTI(xQUEUE, pxQueue)->pcReadFrom = _M_STRUCTI(xQUEUE, pxQueue)->pcHead;
		}
		_M_memcpy(
				(DA_long *) pvBuffer,
				(DA_long *)((_DBIOS_DM_I_mmapped_base + (DA_long)(_M_STRUCTI(xQUEUE, pxQueue)->pcReadFrom))),
				( unsigned ) _M_STRUCTI(xQUEUE, pxQueue)->uxItemSize
				);
	}	
}
/*-----------------------------------------------------------*/

static void prvUnlockQueue( xQueueHandle pxQueue )
{
	--( _M_STRUCTI(xQUEUE, pxQueue)->xTxLock );
	if( _M_STRUCTI(xQUEUE, pxQueue)->xTxLock > queueUNLOCKED )
	{
		_M_STRUCTI(xQUEUE, pxQueue)->xTxLock = queueUNLOCKED;

	}
	--( _M_STRUCTI(xQUEUE, pxQueue)->xRxLock );
	if( _M_STRUCTI(xQUEUE, pxQueue)->xRxLock > queueUNLOCKED )
	{
		_M_STRUCTI(xQUEUE, pxQueue)->xRxLock = queueUNLOCKED;

	}
}
/*-----------------------------------------------------------*/

static signed portBASE_TYPE prvIsQueueEmpty( const xQueueHandle pxQueue )
{
signed portBASE_TYPE xReturn;
	xReturn = ( _M_STRUCTI(xQUEUE, pxQueue)->uxMessagesWaiting == ( unsigned portBASE_TYPE ) 0 );
	return xReturn;
}
/*-----------------------------------------------------------*/

static signed portBASE_TYPE prvIsQueueFull( const xQueueHandle pxQueue )
{
signed portBASE_TYPE xReturn;
	xReturn = ( _M_STRUCTI(xQUEUE, pxQueue)->uxMessagesWaiting == _M_STRUCTI(xQUEUE, pxQueue)->uxLength );
	return xReturn;
}

signed portBASE_TYPE xQueueGenericReceive( xQueueHandle pxQueue, const void * const pvBuffer, portTickType xTicksToWait, portBASE_TYPE xJustPeeking )
{
signed portBASE_TYPE xReturn = pdTRUE;
signed portCHAR *pcOriginalReadPosition;

	/* This function is very similar to xQueueGenericSend().  See comments
	within xQueueGenericSend() for a more detailed explanation.

	/* Capture the current time status for future reference. */
	portTickType currTime;
	portTickType initTime;
	if (xTicksToWait > 0)
		initTime = xTaskGetTickCount();


	/* Make sure interrupts do not access the queue. */
	prvLockQueue( pxQueue );

	do
	{
		/* If there are no messages in the queue we may have to block. */
		if( prvIsQueueEmpty( pxQueue ) )
		{
			/* There are no messages in the queue, do we want to block or just
			leave with nothing? */			
			if( xTicksToWait > ( portTickType ) 0 )
			{
				usleep(ARM_USLEEP_INTERVAL);
				prvUnlockQueue( pxQueue );
				if( _M_STRUCTI(xQUEUE, pxQueue)->uxMessagesWaiting == ( unsigned portBASE_TYPE ) 0 )
				{
					xReturn = errQUEUE_EMPTY;
				}
			}
		}
	
		if( xReturn != errQUEUE_EMPTY )
		{
			printf("qr-- %x\n",_M_STRUCTI(xQUEUE, pxQueue));
			if( _M_STRUCTI(xQUEUE, pxQueue)->uxMessagesWaiting > ( unsigned portBASE_TYPE ) 0 )
			{
				/* Remember our read position in case we are just peeking. */
				pcOriginalReadPosition = _M_STRUCTI(xQUEUE, pxQueue)->pcReadFrom;
				prvCopyDataFromQueue( pxQueue, pvBuffer );
				if( xJustPeeking == pdFALSE )
				{
					/* We are actually removing data. */
					--( _M_STRUCTI(xQUEUE, pxQueue)->uxMessagesWaiting );
						
					/* Increment the lock count so prvUnlockQueue knows to check for
					tasks waiting for space to become available on the queue. */
					++( _M_STRUCTI(xQUEUE, pxQueue)->xRxLock );
					
				}
				else
				{
					/* We are not removing the data, so reset our read
					pointer. */
					_M_STRUCTI(xQUEUE, pxQueue)->pcReadFrom = pcOriginalReadPosition;

					/* The data is being left in the queue, so increment the
					lock count so prvUnlockQueue knows to check for other
					tasks waiting for the data to be available. */
					++( _M_STRUCTI(xQUEUE, pxQueue)->xTxLock );						
				}
				
 				xReturn = pdPASS;					
			}
			else
			{
				xReturn = errQUEUE_EMPTY;
			}
		}

		if( xReturn == errQUEUE_EMPTY )
		{
			if( xTicksToWait > 0 )
			{
				currTime = xTaskGetTickCount();
				if ((currTime - initTime) < xTicksToWait)
				{
					xReturn = queueERRONEOUS_UNBLOCK;
				}
				else{
					printf("ffff %d\n",currTime - initTime);fflush(stdout);
				}
			}
		}
	} while( xReturn == queueERRONEOUS_UNBLOCK );

	/* We no longer require exclusive access to the queue. */
	prvUnlockQueue( pxQueue );
	return xReturn;
}

signed portBASE_TYPE xQueueGenericSend( xQueueHandle pxQueue, const void * const pvItemToQueue, portTickType xTicksToWait, portBASE_TYPE xCopyPosition )
{
signed portBASE_TYPE xReturn = pdPASS;
	/* Capture the current time status for future reference. */
	portTickType currTime;
	portTickType initTime;
	if (xTicksToWait > 0)
		initTime = xTaskGetTickCount();

	/* Make sure interrupts do not access the queue event list. */
	prvLockQueue( pxQueue );
		
	/* If the queue is already full we may have to block. */
	printf("qs-- %ld\n",_M_STRUCTI(xQUEUE, pxQueue));
	do
	{
		if( prvIsQueueFull( pxQueue ) )
		{
			/* The queue is full - do we want to block or just leave without
			posting? */
			if( xTicksToWait > ( portTickType ) 0 )
			{
				usleep(ARM_USLEEP_INTERVAL);
				if( _M_STRUCTI(xQUEUE, pxQueue)->uxMessagesWaiting == _M_STRUCTI(xQUEUE, pxQueue)->uxLength )
				{
					xReturn = errQUEUE_FULL;
				}

				prvLockQueue( pxQueue );				
			}
		}
			
		if( xReturn != errQUEUE_FULL )
		{
			if( _M_STRUCTI(xQUEUE, pxQueue)->uxMessagesWaiting < _M_STRUCTI(xQUEUE, pxQueue)->uxLength )
			{
				/* There is room in the queue, copy the data into the queue. */			
				prvCopyDataToQueue( pxQueue, pvItemToQueue, xCopyPosition );
				xReturn = pdPASS;
	
				/* Update the TxLock count so prvUnlockQueue knows to check for
				tasks waiting for data to become available in the queue. */
				++( _M_STRUCTI(xQUEUE, pxQueue)->xTxLock );
			}
			else
			{
				xReturn = errQUEUE_FULL;
			}
		}

		if( xReturn == errQUEUE_FULL )
		{
			if( xTicksToWait > 0 )
			{
				currTime = xTaskGetTickCount();
				if ((currTime - initTime) < xTicksToWait)
				{
					xReturn = queueERRONEOUS_UNBLOCK;
				}
			}
		}
	}
	while( xReturn == queueERRONEOUS_UNBLOCK );

	prvUnlockQueue( pxQueue );
	return xReturn;
}

xQueueHandle xQueueCreate( unsigned portBASE_TYPE uxQueueLength, unsigned portBASE_TYPE uxItemSize )
{
xQUEUE *pxNewQueue;
size_t xQueueSizeInBytes;

	/* Allocate the new queue structure. */
	if( uxQueueLength > ( unsigned portBASE_TYPE ) 0 )
	{
		pxNewQueue = ( xQUEUE * ) DSPAbstractRM_AllocMemory( sizeof( xQUEUE ) );
		if( pxNewQueue != NULL )
		{
			/* Create the list of pointers to queue items.  The queue is one byte
			longer than asked for to make wrap checking easier/faster. */
			xQueueSizeInBytes = ( size_t ) ( uxQueueLength * uxItemSize ) + ( size_t ) 1;

			_M_STRUCTI(xQUEUE, pxNewQueue)->pcHead = ( signed portCHAR * ) DSPAbstractRM_AllocMemory( xQueueSizeInBytes );
			if( _M_STRUCTI(xQUEUE, pxNewQueue)->pcHead != NULL )
			{
				/* Initialise the queue members as described above where the
				queue type is defined. */
				_M_STRUCTI(xQUEUE, pxNewQueue)->pcTail = _M_STRUCTI(xQUEUE, pxNewQueue)->pcHead + ( uxQueueLength * uxItemSize );
				_M_STRUCTI(xQUEUE, pxNewQueue)->uxMessagesWaiting = 0;
				_M_STRUCTI(xQUEUE, pxNewQueue)->pcWriteTo = _M_STRUCTI(xQUEUE, pxNewQueue)->pcHead;
				_M_STRUCTI(xQUEUE, pxNewQueue)->pcReadFrom = _M_STRUCTI(xQUEUE, pxNewQueue)->pcHead + ( ( uxQueueLength - 1 ) * uxItemSize );
				_M_STRUCTI(xQUEUE, pxNewQueue)->uxLength = uxQueueLength;
				_M_STRUCTI(xQUEUE, pxNewQueue)->uxItemSize = uxItemSize;
				_M_STRUCTI(xQUEUE, pxNewQueue)->xRxLock = queueUNLOCKED;
				_M_STRUCTI(xQUEUE, pxNewQueue)->xTxLock = queueUNLOCKED;

				return  pxNewQueue;
			}
			else
			{
				DSPAbstractRM_FreeMemory( pxNewQueue );
			}
		}
	}

	/* Will only reach here if we could not allocate enough memory or no memory
	was required. */
	return NULL;
}


#ifdef __cplusplus
}
#endif

