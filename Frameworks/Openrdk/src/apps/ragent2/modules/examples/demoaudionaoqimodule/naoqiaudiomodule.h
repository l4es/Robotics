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

#ifndef RDK_MODULE_NAOQIAUDIOMODULE
#define RDK_MODULE_NAOQIAUDIOMODULE

#include <rdkcore/modules/module.h>
#include <rdkcore/sharedmemory/sharedmemory.h>
#include <nao-objects/sensordatashm.h>

namespace RDK2 { namespace RAgent {

#define EVENT_HANDLER_CLASS NaoQiAudioModule

	class NaoQiAudioModule : public Module {
		public:
			NaoQiAudioModule(): shmem(NULL) { }
			virtual ~NaoQiAudioModule()
			{
				if (shmem != NULL)
				{
					delete shmem;
					shmem = NULL;
				}
			}


			bool initConfigurationProperties();
			bool init();
			void exec();
			void exitRequested();
			//void cleanup();

		private:
			SharedMemory* shmem;

			bool doSayToShm(const Event* e);
			DECLARE_EVENT_HANDLER(doSayToShm);
			void sayToShm(RdkAudioData* data);
	};

}} // namespace

#endif
