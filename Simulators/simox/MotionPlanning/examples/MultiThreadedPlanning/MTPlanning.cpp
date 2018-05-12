
#include "MTPlanningWindow.h"


#include <string.h>
#include <iostream>
using namespace std;
using namespace VirtualRobot;


void startMTPlanning()
{
    MTPlanningWindow* agfw = new MTPlanningWindow();
    agfw->main();
    delete agfw;
}

int main(int argc, char** argv)
{
    SoDB::init();
    SoQt::init(argc, argv, "MT");
    cout << " --- START --- " << endl;

    if (!CollisionChecker::IsSupported_Multithreading_MultipleColCheckers())
    {
        cout << " The collision detection library that is linked to simox does not support multi threading. Aborting...." << endl;
        return -1;
    }

#ifdef WIN32
    cout << "Visual Studio users: Be sure to start this example with <ctrl>+F5 (RELEASE mode) for full performance (otherwise only 1 thread will be used)" << endl;
#endif

    try
    {
        startMTPlanning();
    }
    catch (std::exception e)
    {
        std::cout << "Exception: " << e.what() << std::endl ;
    }
    catch (...)
    {
        ;
    }

    cout << " --- END --- " << endl;

    return 0;
}
