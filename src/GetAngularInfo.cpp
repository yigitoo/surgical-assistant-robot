#include "KinovaTypes.h"
#include <iostream>
#ifdef __linux__ 
#include <dlfcn.h>
#include <vector>
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include <stdio.h>
#include <unistd.h>
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
int(*MyGetAngularCommand)(AngularPosition &);
int(*MyGetAngularPosition)(AngularPosition &);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);
int (*MyGetActuatorAcceleration)(AngularAcceleration &Response);
int (*MyGetAngularVelocity)(AngularPosition &Response);


int main(int argc, char *argv[])
{
	int result;
	AngularPosition dataCommand;
	AngularPosition dataPosition;

	int programResult = 0;

#ifdef __linux__ 
	//We load the API.
	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	//We load the functions from the library
	MyInitAPI = (int(*)()) dlsym(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "CloseAPI");
	MyGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
	MyGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");
	MyGetActuatorAcceleration = (int(*)(AngularAcceleration &)) dlsym(commandLayer_handle, "GetActuatorAcceleration");
	MyGetAngularVelocity = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularVelocity");
#elif _WIN32
	//We load the API.
	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

	//We load the functions from the library
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyGetAngularCommand = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularCommand");
	MyGetAngularPosition = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularPosition");
	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) GetProcAddress(commandLayer_handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice devices)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
	MyGetActuatorAcceleration = (int(*)(AngularAcceleration &)) GetProcAddress(commandLayer_handle, "GetActuatorAcceleration");
	MyGetAngularVelocity = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularVelocity");
#endif

	
	//Verify that all functions has been loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) || (MyGetAngularPosition == NULL)
		|| (MySetActiveDevice == NULL) || (MyGetDevices == NULL) || (MyGetAngularVelocity == NULL))
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

			(*MyGetAngularCommand)(dataCommand);
			(*MyGetAngularPosition)(dataPosition);

			cout << "*********************************" << endl;
			cout << "Actuator 1   command : " << dataCommand.Actuators.Actuator1 << " deg" << "     Position : " << dataPosition.Actuators.Actuator1 << " deg" << endl;
			cout << "Actuator 2   command : " << dataCommand.Actuators.Actuator2 << " deg" << "     Position : " << dataPosition.Actuators.Actuator2 << " deg" << endl;
			cout << "Actuator 3   command : " << dataCommand.Actuators.Actuator3 << " deg" << "     Position : " << dataPosition.Actuators.Actuator3 << " deg" << endl;
			cout << "Actuator 4   command : " << dataCommand.Actuators.Actuator4 << " deg" << "     Position : " << dataPosition.Actuators.Actuator4 << " deg" << endl;
			cout << "Actuator 5   command : " << dataCommand.Actuators.Actuator5 << " deg" << "     Position : " << dataPosition.Actuators.Actuator5 << " deg" << endl;
			cout << "Actuator 6   command : " << dataCommand.Actuators.Actuator6 << " deg" << "     Position : " << dataPosition.Actuators.Actuator6 << " deg" << endl;
			cout << "Actuator 7   command : " << dataCommand.Actuators.Actuator7 << " deg" << "     Position : " << dataPosition.Actuators.Actuator7 << " deg" << endl << endl;

			cout << "  Finger 1   command: " << dataCommand.Fingers.Finger1 << "     Position : " << dataPosition.Fingers.Finger1 << endl;
			cout << "  Finger 2   command: " << dataCommand.Fingers.Finger2 << "     Position : " << dataPosition.Fingers.Finger2 << endl;
			cout << "  Finger 3   command: " << dataCommand.Fingers.Finger3 << "     Position : " << dataPosition.Fingers.Finger3 << endl;
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
