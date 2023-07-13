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
int(*MyGetCartesianCommand)(CartesianPosition &);
int(*MyGetCartesianPosition)(CartesianPosition &);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);

int main(int argc, char* argv[])
{
	int result;
	CartesianPosition dataCommand;
	CartesianPosition dataPosition;

	int programResult = 0;

#ifdef __linux__ 
	//We load the API.
	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	//We load the functions from the library
	MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
	MyGetCartesianCommand = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianCommand");
	MyGetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianPosition");
	MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
	MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
#elif _WIN32
	//We load the API.
	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

	//We load the functions from the library
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");
	MyGetCartesianPosition = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianPosition");
	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) GetProcAddress(commandLayer_handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice devices)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
#endif

	//If the API was loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetCartesianCommand == NULL) || (MyGetDevices == NULL)
		|| (MyGetCartesianPosition == NULL) || (MySetActiveDevice == NULL))
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

			(*MyGetCartesianCommand)(dataCommand);
			(*MyGetCartesianPosition)(dataPosition);

			cout << "*********************************" << endl;
			cout << "X   command : " << dataCommand.Coordinates.X << " m" << "     Position : " << dataPosition.Coordinates.X << " m" << endl;
			cout << "Y   command : " << dataCommand.Coordinates.Y << " m" << "     Position : " << dataPosition.Coordinates.Y << " m" << endl;
			cout << "Z   command : " << dataCommand.Coordinates.Z << " m" << "     Position : " << dataPosition.Coordinates.Z << " m" << endl;
			cout << "Theta X   command : " << dataCommand.Coordinates.ThetaX << " Rad" << "     Position : " << dataPosition.Coordinates.ThetaX << " Rad" << endl;
			cout << "Theta Y   command : " << dataCommand.Coordinates.ThetaY << " Rad" << "     Position : " << dataPosition.Coordinates.ThetaY << " Rad" << endl;
			cout << "Theta Z   command : " << dataCommand.Coordinates.ThetaZ << " Rad" << "     Position : " << dataPosition.Coordinates.ThetaZ << " Rad" << endl << endl;

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
