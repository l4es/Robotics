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

#ifndef  RDK_SHAREDMEMORY_TRAITSNAO_INC
#define  RDK_SHAREDMEMORY_TRAITSNAO_INC

#include "naojoints.h"
#include <string>

static const std::string MOTION_SHM_NAME = "MotionSharedMemory";
static const std::string IMAGE_SHM_NAME = "ImageSharedMemory";

static const size_t JOINTS_VALUES_SIZE = Nao::NaoJoints::NAO_JOINTS_COUNT;
static const size_t FORCE_SENSOR_SIZE  = 8;
static const size_t CAMERAMATRIX_SIZE  = 12; // R=3x3 T=3x1 -> M=3x4


#endif   /* ----- #ifndef SHAREDMEMORY_TRAITS_INC  ----- */
