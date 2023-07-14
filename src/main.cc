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

#include "api.hpp"

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
int cartesian_control(int actuator_id, int result);
int get_actuator_current(int actuator_id, int result);
int get_angular_info(int actuator_id, int result);
int get_cartesian_info(int actuator_id, int result);
int get_temperature(int actuator_id, int result);
int get_torque_value(int actuator_id, int result);
int InitializeAPIFunctions();
int main(int argc, char *argv[]);

int main(int argc, char *argv[])
{
	int option = 2, actuator_id = 1;
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
		programResult = cartesian_control(actuator_id, result);
	} else if (option == 4)
	{
		programResult = get_actuator_current(actuator_id, result);
	} else if (option == 5)
	{
		programResult = get_angular_info(actuator_id, result);
	} else if (option == 6)
	{
		programResult = get_cartesian_info(actuator_id, result);
	} else if (option == 7) 
	{
		programResult = get_temperature(actuator_id, result);
	} else if (option == 8)
	{
		programResult = get_torque_value(actuator_id, result);
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

int admittance(int actuator_id, int result)
{
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
	AngularPosition currentCommand;

	KinovaDevice list[MAX_KINOVA_DEVICE];

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

		pointToSend.Position.Actuators.Actuator1 = 48;
		pointToSend.Position.Actuators.Actuator2 = 48;
		pointToSend.Position.Actuators.Actuator3 = 48;
		pointToSend.Position.Actuators.Actuator4 = 48;
		pointToSend.Position.Actuators.Actuator5 = 48;
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

int cartesian_control(int actuator_id, int result)
{
    CartesianPosition currentCommand;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		cout << "Send the robot to HOME position" << endl;
		MyMoveHome();

		cout << "Initializing the fingers" << endl;
		MyInitFingers();

		TrajectoryPoint pointToSend;
		pointToSend.InitStruct();

		//We specify that this point will be used an angular(joint by joint) velocity vector.
		pointToSend.Position.Type = CARTESIAN_VELOCITY;

		pointToSend.Position.CartesianPosition.X = 0;
		pointToSend.Position.CartesianPosition.Y = -0.15; //Move along Y axis at 20 cm per second
		pointToSend.Position.CartesianPosition.Z = 0;
		pointToSend.Position.CartesianPosition.ThetaX = 0;
		pointToSend.Position.CartesianPosition.ThetaY = 0;
		pointToSend.Position.CartesianPosition.ThetaZ = 0;

		pointToSend.Position.Fingers.Finger1 = 0;
		pointToSend.Position.Fingers.Finger2 = 0;
		pointToSend.Position.Fingers.Finger3 = 0;

		for (int i = 0; i < 200; i++)
		{
			//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
			MySendBasicTrajectory(pointToSend);
			sleep(5);	
		}

		pointToSend.Position.CartesianPosition.Y = 0;
		pointToSend.Position.CartesianPosition.Z = 0.1;

		for (int i = 0; i < 200; i++)
		{
			//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
			MySendBasicTrajectory(pointToSend);
			sleep(5);	
		}

		cout << "Send the robot to HOME position" << endl;
		MyMoveHome();

		//We specify that this point will be an angular(joint by joint) position.
		pointToSend.Position.Type = CARTESIAN_POSITION;

		//We get the actual angular command of the robot.
		MyGetCartesianCommand(currentCommand);

		pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X;
		pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y - 0.1f;
		pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z;
		pointToSend.Position.CartesianPosition.ThetaX = currentCommand.Coordinates.ThetaX;
		pointToSend.Position.CartesianPosition.ThetaY = currentCommand.Coordinates.ThetaY;
		pointToSend.Position.CartesianPosition.ThetaZ = currentCommand.Coordinates.ThetaZ;

		cout << "*********************************" << endl;
		cout << "Sending the first point to the robot." << endl;
		MySendBasicTrajectory(pointToSend);

		pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z + 0.1f;
		cout << "Sending the second point to the robot." << endl;
		MySendBasicTrajectory(pointToSend);

		cout << "*********************************" << endl << endl << endl;
	}
	return 1;
}

int get_actuator_current(int actuator_id, int result)
{
	AngularPosition current;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		MyGetAngularCurrent(current);

		cout << "*********************************" << endl;
		cout << "Actuator 1   current : " << current.Actuators.Actuator1 << " A" << endl;
		cout << "Actuator 2   current : " << current.Actuators.Actuator2 << " A" << endl;
		cout << "Actuator 3   current : " << current.Actuators.Actuator3 << " A" << endl;
		cout << "Actuator 4   current : " << current.Actuators.Actuator4 << " A" << endl;
		cout << "Actuator 5   current : " << current.Actuators.Actuator5 << " A" << endl;
		cout << "Actuator 6   current : " << current.Actuators.Actuator6 << " A" << endl;
		cout << "Actuator 7   current : " << current.Actuators.Actuator7 << " A" << endl;
		cout << "*********************************" << endl << endl << endl;
	}
	return 1;
}

int get_angular_info(int actuator_id, int result)
{
	AngularPosition dataCommand;
	AngularPosition dataPosition;

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

	return 1;
}

int get_cartesian_info(int actuator_id, int result)
{
	CartesianPosition dataCommand;
	CartesianPosition dataPosition;

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

	return 1;
}

int get_temperature(int actuator_id, int result)
{
	GeneralInformations data;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		MyGetGeneralInformations(data);

		cout << "*********************************" << endl;
		cout << "Actuator 1 temperature : " << data.ActuatorsTemperatures[0] << " °C" << endl;
		cout << "Actuator 2 temperature : " << data.ActuatorsTemperatures[1] << " °C" << endl;
		cout << "Actuator 3 temperature : " << data.ActuatorsTemperatures[2] << " °C" << endl;
		cout << "Actuator 4 temperature : " << data.ActuatorsTemperatures[3] << " °C" << endl;
		cout << "Actuator 5 temperature : " << data.ActuatorsTemperatures[4] << " °C" << endl;
		cout << "Actuator 6 temperature : " << data.ActuatorsTemperatures[5] << " °C" << endl;
		cout << "Actuator 7 temperature : " << data.ActuatorsTemperatures[6] << " °C" << endl;
		cout << "*********************************" << endl << endl << endl;
	}

	return 1;
}

int get_torque_value(int actuator_id, int result)
{
	AngularPosition torque;
	AngularPosition torqueGravityFree;

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
		RTLD_NOW | RTLD_GLOBAL
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