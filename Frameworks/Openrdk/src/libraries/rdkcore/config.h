#ifdef OpenRDK_ARCH_GEODE
#include "config-cross-geode.h"
#elif OpenRDK_ARCH_ATOM
#include "config-cross-atom.h"
#elif OpenRDK_ARCH_ARM9
#include "config-cross-arm9.h"
#else
#include "config-generic.h"
#endif
