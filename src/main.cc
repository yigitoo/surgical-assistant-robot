//Importing Kinova SDK API.
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
// ADMITANCE
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MyStartForceControl)();
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);

// ANGULAR_CONTROL
int(*MySendBasicTrajectory)(TrajectoryPoint command);
int(*MyMoveHome)();
int(*MyInitFingers)();
int(*MyGetAngularCommand)(AngularPosition &);

// CARTESIAN_CONTROL
int(*MyGetCartesianCommand)(CartesianPosition &);

// ACTUATOR_CURRENT
int(*MyGetAngularCurrent)(AngularPosition &Response);

// ANGULAR_INFO
int(*MyGetAngularPosition)(AngularPosition &);
int (*MyGetActuatorAcceleration)(AngularAcceleration &Response);
int (*MyGetAngularVelocity)(AngularPosition &Response);

// CARTEISAN_INFO
int(*MyGetCartesianPosition)(CartesianPosition &);

// GET_TEMPERATURE
int(*MyGetGeneralInformations)(GeneralInformations &Response);

// GET_TORQUE_VALUE
int(*MyGetAngularForce)(AngularPosition &Response);
int(*MyGetAngularForceGravityFree)(AngularPosition &Response);

void sleep(int duration);
int admittance(int actuator_id, int result);
int angular_control(int actuator_id, int result);
int InitializeAPIFunctions();
int main(void);

int main()
{
	int option = 1, actuator_id = 1;
	int result, programResult;
    int is_initialized = InitializeAPIFunctions();
    if(!is_initialized) return 1;
	
	result = (*MyInitAPI)();

	cout << "Initialization's result :" << result << endl;
	if (result == 1015)
	{
		cout << "The device has not been found. Please open and setup device for usage." << endl;
	}
	if (option == 1)
	{
		programResult = admittance(actuator_id, result);
	} else if (option == 2)
	{
		programResult = angular_control(actuator_id, result);
	} else if (option == 3)
	{

	} else if (option == 4)
	{

	} else {
		cout << "Error: Unknown option" << endl;
		return 1;
	}
	cout << endl << "C L O S I N G   A P I" << endl;
	result = (*MyCloseAPI)();
	#ifdef __linux__ 
		dlclose(commandLayer_handle);
	#elif _WIN32
		FreeLibrary(commandLayer_handle);
	#endif
    return !programResult;
}

int admittance(int actuator_id, int result) {
	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
	cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

	//Setting the current device as the active device.
	MySetActiveDevice(list[i]);

	MyStartForceControl();

	cout << endl << "The robot's admittance control is now active, you can move it freely with your hand." <<
	" To deactivate it call the function StopForceControl()." << endl;
	}
	
	return 1;
}

int angular_control(int actuator_id, int result)
{
	KinovaDevice list[MAX_KINOVA_DEVICE];
	AngularPosition currentCommand;

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		std::cout << "Send the robot to HOME position" << std::endl;
		MyMoveHome();

		std::cout << "Initializing the motors." << std::endl;
		MyInitFingers();

		TrajectoryPoint pointToSend;
		pointToSend.InitStruct();

		//We specify that this point will be used an angular(joint by joint) velocity vector.
		pointToSend.Position.Type = ANGULAR_VELOCITY;

		pointToSend.Position.Actuators.Actuator1 = 0;
		pointToSend.Position.Actuators.Actuator2 = 0;
		pointToSend.Position.Actuators.Actuator3 = 0;
		pointToSend.Position.Actuators.Actuator4 = 0;
		pointToSend.Position.Actuators.Actuator5 = 0;
		pointToSend.Position.Actuators.Actuator6 = 48; //joint 6 at 48 degrees per second.

		pointToSend.Position.Fingers.Finger1 = 0;
		pointToSend.Position.Fingers.Finger2 = 0;
		pointToSend.Position.Fingers.Finger3 = 0;

		for (int i = 0; i < 300; i++)
		{
			//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
			MySendBasicTrajectory(pointToSend);
			sleep(5);
		}

		pointToSend.Position.Actuators.Actuator6 = -20; //joint 6 at -20 degrees per second.

		for (int i = 0; i < 300; i++)
		{
			//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
			MySendBasicTrajectory(pointToSend);
			sleep(5);
		}

		std::cout << "Send the robot to HOME position" << std::endl;
		MyMoveHome();

		//We specify that this point will be an angular(joint by joint) position.
		pointToSend.Position.Type = ANGULAR_POSITION;

		//We get the actual angular command of the robot.
		MyGetAngularCommand(currentCommand);

		pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1 + 30;
		pointToSend.Position.Actuators.Actuator2 = currentCommand.Actuators.Actuator2;
		pointToSend.Position.Actuators.Actuator3 = currentCommand.Actuators.Actuator3;
		pointToSend.Position.Actuators.Actuator4 = currentCommand.Actuators.Actuator4;
		pointToSend.Position.Actuators.Actuator5 = currentCommand.Actuators.Actuator5;
		pointToSend.Position.Actuators.Actuator6 = currentCommand.Actuators.Actuator6;

		std::cout << "*********************************" << std::endl;
		std::cout << "Sending the first point to the robot." << std::endl;
		MySendBasicTrajectory(pointToSend);

		pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1 - 60;
		std::cout << "Sending the second point to the robot." << std::endl;
		MySendBasicTrajectory(pointToSend);

		pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1;
		std::cout << "Sending the third point to the robot." << std::endl;
		MySendBasicTrajectory(pointToSend);

		std::cout << "*********************************" << std::endl << std::endl << std::endl;
	}

	std::cout << std::endl << "WARNING: Your robot is now set to angular control. If you use the joystick, it will be a joint by joint movement." << std::endl;

	return 1;
}

void sleep(int duration) {
#ifdef __linux__ 
	usleep(duration * 1000);
#elif _WIN32
	Sleep(duration);	
#endif
}

int InitializeAPIFunctions(){
    int programResult;
#ifdef __linux__ 
	commandLayer_handle = dlopen(
		"Kinova.API.USBCommandLayerUbuntu.so",
		RTLD_NOW|RTLD_GLOBAL
	);

	//We load the functions from the library
    // ADMITANCE
	MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
	MyStartForceControl = (int (*)()) dlsym(commandLayer_handle,"StartForceControl");
	MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
	MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");

	// ANGULAR_CONTROL
	MyMoveHome = (int (*)()) dlsym(commandLayer_handle,"MoveHome");
	MyInitFingers = (int (*)()) dlsym(commandLayer_handle,"InitFingers");
	MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
	MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCommand");

	// CARTESIAN_CONTROL
	MyGetCartesianCommand = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianCommand");

	// ACTUATOR_CURRENT
	MyGetAngularCurrent = (int (*)(AngularPosition &Response)) dlsym(commandLayer_handle,"GetAngularCurrent");

	// ANGULAR_INFO
	MyGetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
	MyGetActuatorAcceleration = (int (*)(AngularAcceleration &)) dlsym(commandLayer_handle, "GetActuatorAcceleration");
	MyGetAngularVelocity = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularVelocity");

	// CARTEISAN_INFO
	MyGetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianPosition");

	// GET_TEMPERATURE
	MyGetGeneralInformations = (int (*)(GeneralInformations &info)) dlsym(commandLayer_handle,"GetGeneralInformations");

	// GET_TORQUE_VALUE
	MyGetAngularForce = (int (*)(AngularPosition &Response)) dlsym(commandLayer_handle,"GetAngularForce");
	MyGetAngularForceGravityFree = (int (*)(AngularPosition &Response)) dlsym(commandLayer_handle,"GetAngularForceGravityFree");
#elif _WIN32
	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

	//We load the functions from the library
    // ADMITANCE
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyStartForceControl = (int(*)()) GetProcAddress(commandLayer_handle, "StartForceControl");
	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) GetProcAddress(commandLayer_handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice devices)) GetProcAddress(commandLayer_handle, "SetActiveDevice");

	// ANGULAR_CONTROL
	MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
	MyGetAngularCommand = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularCommand");
	MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
	MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");

	// CARTESIAN_CONTROL
	MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");

	// ACTUATOR_CURRENT
	MyGetAngularCurrent = (int(*)(AngularPosition &Response)) GetProcAddress(commandLayer_handle, "GetAngularCurrent");

	// ANGULAR_INFO
	MyGetAngularPosition = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularPosition");
	MyGetActuatorAcceleration = (int(*)(AngularAcceleration &)) GetProcAddress(commandLayer_handle, "GetActuatorAcceleration");
	MyGetAngularVelocity = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularVelocity");

	// CARTEISAN_INFO
	MyGetCartesianPosition = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianPosition");

	// GET_TEMPERATURE
	MyGetGeneralInformations = (int(*)(GeneralInformations &info)) GetProcAddress(commandLayer_handle, "GetGeneralInformations");

	// GET_TORQUE_VALUE
	MyGetAngularForce = (int(*)(AngularPosition &Response)) GetProcAddress(commandLayer_handle, "GetAngularForce");
	MyGetAngularForceGravityFree = (int(*)(AngularPosition &Response)) GetProcAddress(commandLayer_handle, "GetAngularForceGravityFree");
#endif

    //If the API was loaded correctly
	if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetDevices == NULL) || (MySetActiveDevice == NULL)
	|| (MyStartForceControl == NULL) 
	|| (MySendBasicTrajectory == NULL) || (MyGetAngularCommand == NULL) || (MyMoveHome == NULL) || (MyInitFingers == NULL)
	|| (MyGetCartesianCommand == NULL)
	|| (MyGetAngularCurrent == NULL)
	|| (MyGetAngularPosition == NULL) || (MyGetActuatorAcceleration == NULL) || (MyGetAngularVelocity == NULL)
	|| (MyGetCartesianPosition == NULL)
	|| (MyGetGeneralInformations == NULL)
	|| (MyGetAngularForce == NULL) || (MyGetAngularForceGravityFree == NULL))
	{
		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
		programResult = 0;
	}
	else
	{
		cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;
		programResult = 1;

	}
    return programResult;
}