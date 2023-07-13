#include "KinovaTypes.h"
#include <iostream>
#ifdef __linux__ 
#include <dlfcn.h>
#include <vector>
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include <stdio.h>
#elif _WIN32
#include <Windows.h>
#include "CommunicationLayer.h"
#include "CommandLayer.h"
#include <conio.h>
#endif

using namespace std;

//A handle to the API.
#ifdef __linux__ 
void * commandLayer_handle;
#elif _WIN32
HINSTANCE commandLayer_handle;
#endif

//Function pointers to the functions we need
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MyGetAngularForce)(AngularPosition &Response);
int(*MyGetAngularForceGravityFree)(AngularPosition &Response);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);

int main(int argc, char* argv[])
{
	int result;
	AngularPosition torque;
	AngularPosition torqueGravityFree;

	int programResult = 0;

#ifdef __linux__ 
	//We load the API.
	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	//We load the functions from the library
	MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
	MyGetAngularForce = (int (*)(AngularPosition &Response)) dlsym(commandLayer_handle,"GetAngularForce");
	MyGetAngularForceGravityFree = (int (*)(AngularPosition &Response)) dlsym(commandLayer_handle,"GetAngularForceGravityFree");
	MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
	MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
#elif _WIN32
	//We load the API.
	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

	//We load the functions from the library
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyGetAngularForce = (int(*)(AngularPosition &Response)) GetProcAddress(commandLayer_handle, "GetAngularForce");
	MyGetAngularForceGravityFree = (int(*)(AngularPosition &Response)) GetProcAddress(commandLayer_handle, "GetAngularForceGravityFree");
	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) GetProcAddress(commandLayer_handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice devices)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
#endif

	//If the API was loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetDevices == NULL)
		|| (MySetActiveDevice == NULL) || (MyGetAngularForce == NULL) || (MyGetAngularForceGravityFree == NULL))
	{
		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
		programResult = 0;
	}
	else
	{
		cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

		result = (*MyInitAPI)();

		cout << "Initialization's result :" << result << endl;

		KinovaDevice list[MAX_KINOVA_DEVICE];

		int devicesCount = MyGetDevices(list, result);

		for (int i = 0; i < devicesCount; i++)
		{
			cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

			//Setting the current device as the active device.
			MySetActiveDevice(list[i]);

			MyGetAngularForce(torque);
			MyGetAngularForceGravityFree(torqueGravityFree);
			cout << "*********************************" << endl;
			cout << "Actuator 1   torque : " << torque.Actuators.Actuator1 << " N*m" << "     without gravity : " << torqueGravityFree.Actuators.Actuator1 << " N*m" << endl;
			cout << "Actuator 2   torque : " << torque.Actuators.Actuator2 << " N*m" << "     without gravity : " << torqueGravityFree.Actuators.Actuator2 << " N*m" << endl;
			cout << "Actuator 3   torque : " << torque.Actuators.Actuator3 << " N*m" << "     without gravity : " << torqueGravityFree.Actuators.Actuator3 << " N*m" << endl;
			cout << "Actuator 4   torque : " << torque.Actuators.Actuator4 << " N*m" << "     without gravity : " << torqueGravityFree.Actuators.Actuator4 << " N*m" << endl;
			cout << "Actuator 5   torque : " << torque.Actuators.Actuator5 << " N*m" << "     without gravity : " << torqueGravityFree.Actuators.Actuator5 << " N*m" << endl;
			cout << "Actuator 6   torque : " << torque.Actuators.Actuator6 << " N*m" << "     without gravity : " << torqueGravityFree.Actuators.Actuator6 << " N*m" << endl;
			cout << "Actuator 7   torque : " << torque.Actuators.Actuator7 << " N*m" << "     without gravity : " << torqueGravityFree.Actuators.Actuator7 << " N*m" << endl;
			cout << "*********************************" << endl << endl << endl;
		}

		cout << endl << "C L O S I N G   A P I" << endl;
		result = (*MyCloseAPI)();

		programResult = 1;
	}

#ifdef __linux__ 
	dlclose(commandLayer_handle);
#elif _WIN32
	FreeLibrary(commandLayer_handle);
#endif

	return programResult;

}
