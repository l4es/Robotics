/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef H_RDATA_RDATAQUEUE_ROBJECTQUEUE
#define H_RDATA_RDATAQUEUE_ROBJECTQUEUE

#include <rdkcore/object/object.h>

//#ifndef CYGWIN
#include <rdkcore/posixqueues/pipequeue.h>
//#else
//#include <rdkcore/posixqueues/lockingqueue.h>
//#endif

namespace RDK2 { namespace RepositoryNS {

//#ifndef CYGWIN

struct RObjectPipeQueue : public RDK2::Object, public PosixQueues::PipeQueue<RDK2::Object> 
{
	RObjectPipeQueue(cstr name = "") : PosixQueues::PipeQueue<RDK2::Object>(name) { }
};
	
typedef PosixQueues::PipeQueueConsumer<RDK2::Object> RConsumerObjectPipeQueue;

//#else
//
//struct RObjectPipeQueue : public RDK2::Object, public PosixQueues::LockingQueue<RDK2::Object> 
//{
//	RObjectPipeQueue(cstr name = "") : PosixQueues::LockingQueue<RDK2::Object>(name) { }
//};
//	
//typedef PosixQueues::LockingQueueConsumer<RDK2::Object> RConsumerObjectPipeQueue;
//
//#endif

}} // namespaces

#endif

