#include <iostream>
#include <string>
#include <vector>
#include <chrono>

#include <zmq.hpp>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>


#ifdef __linux__
#include <unistd.h>
#elif _WIN32
#include <Windows.h>
#endif

#include <ctime>

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
// KINOVA K-75+ & K-58 ACTUATORS API functions ____ SECTION 1 OF CODE
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

int admittance_control(int actuator_id, int result)
{
	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);
	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		MyStartForceControl();

		std::cout << std::endl << "The robot's admittance control is now active, you can move it freely with your hand." <<
		" To deactivate it call the function StopForceControl()." << devicesCount << std::endl;
		}
		
	return 1;
}

std::vector<float> angular_control(int actuator_id, int result)
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

		pointToSend.Position.Actuators.Actuator1 = 0;
		pointToSend.Position.Actuators.Actuator2 = 0;
		pointToSend.Position.Actuators.Actuator3 = 0;
		pointToSend.Position.Actuators.Actuator4 = 0;
		pointToSend.Position.Actuators.Actuator5 = 0;
		pointToSend.Position.Actuators.Actuator6 = 0; //joint 6 at 48 degrees per second.

		pointToSend.Position.Fingers.Finger1 = 0;
		pointToSend.Position.Fingers.Finger2 = 0;
		pointToSend.Position.Fingers.Finger3 = 0;

		for (int i = 0; i < 300; i++)
		{
			//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
			MySendBasicTrajectory(pointToSend);
			sleep(5);
		}


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

		pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1 + 3;
		pointToSend.Position.Actuators.Actuator2 = currentCommand.Actuators.Actuator2 + 3;
		pointToSend.Position.Actuators.Actuator3 = currentCommand.Actuators.Actuator3 + 3;
		pointToSend.Position.Actuators.Actuator4 = currentCommand.Actuators.Actuator4 + 3;
		pointToSend.Position.Actuators.Actuator5 = currentCommand.Actuators.Actuator5 + 3;
		pointToSend.Position.Actuators.Actuator6 = currentCommand.Actuators.Actuator6 + 3;

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

		std::cout << std::endl << "WARNING: Your robot is now set to angular control. If you use the joystick, it will be a joint by joint movement." << std::endl;
		
		std::vector<float> angles;
		angles.push_back(pointToSend.Position.Actuators.Actuator1);
		angles.push_back(pointToSend.Position.Actuators.Actuator2);
		angles.push_back(pointToSend.Position.Actuators.Actuator3);
		angles.push_back(pointToSend.Position.Actuators.Actuator4);
		angles.push_back(pointToSend.Position.Actuators.Actuator5);
		angles.push_back(pointToSend.Position.Actuators.Actuator6);
		angles.push_back(pointToSend.Position.Actuators.Actuator7);
		return angles;
	}

	std::vector<float> null;
	return null;
}

CartesianInfo cartesian_control(int result)
{
    CartesianPosition currentCommand;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		std::cout << "Send the robot to HOME position" << std::endl;
		MyMoveHome();

		std::cout << "Initializing the fingers" << std::endl;
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

		std::cout << "Send the robot to HOME position" << std::endl;
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

		std::cout << "*********************************" << std::endl;
		std::cout << "Sending the first point to the robot." << std::endl;
		MySendBasicTrajectory(pointToSend);

		pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z + 0.1f;
		std::cout << "Sending the second point to the robot." << std::endl;
		MySendBasicTrajectory(pointToSend);

		std::cout << "*********************************" << std::endl << std::endl << std::endl;

		return pointToSend.Position.CartesianPosition;
	}

	CartesianInfo null;
	return null;
}

float get_actuator_current(int actuator_id, int result)
{
	AngularPosition current;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		MyGetAngularCurrent(current);

		std::cout << "*********************************" << std::endl;
		std::cout << "Actuator 1   current : " << current.Actuators.Actuator1 << " A" << std::endl;
		std::cout << "Actuator 2   current : " << current.Actuators.Actuator2 << " A" << std::endl;
		std::cout << "Actuator 3   current : " << current.Actuators.Actuator3 << " A" << std::endl;
		std::cout << "Actuator 4   current : " << current.Actuators.Actuator4 << " A" << std::endl;
		std::cout << "Actuator 5   current : " << current.Actuators.Actuator5 << " A" << std::endl;
		std::cout << "Actuator 6   current : " << current.Actuators.Actuator6 << " A" << std::endl;
		std::cout << "Actuator 7   current : " << current.Actuators.Actuator7 << " A" << std::endl;
		std::cout << "*********************************" << std::endl << std::endl << std::endl;
	}
	switch(actuator_id) {
		case 1:
            return current.Actuators.Actuator1;
            break;
        case 2:
            return current.Actuators.Actuator2;
            break;
        case 3:
            return current.Actuators.Actuator3;
            break;
        case 4:
            return current.Actuators.Actuator4;
            break;
        case 5:
            return current.Actuators.Actuator5;
            break;
        case 6:
            return current.Actuators.Actuator6;
            break;
        case 7:
            return current.Actuators.Actuator7;
            break;
        default:
            return -9999;
            break;
	}
}

std::vector<float> get_actuator_currents(int result)
{
	AngularPosition current;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		MyGetAngularCurrent(current);

		std::cout << "*********************************" << std::endl;
		std::cout << "Actuator 1   current : " << current.Actuators.Actuator1 << " A" << std::endl;
		std::cout << "Actuator 2   current : " << current.Actuators.Actuator2 << " A" << std::endl;
		std::cout << "Actuator 3   current : " << current.Actuators.Actuator3 << " A" << std::endl;
		std::cout << "Actuator 4   current : " << current.Actuators.Actuator4 << " A" << std::endl;
		std::cout << "Actuator 5   current : " << current.Actuators.Actuator5 << " A" << std::endl;
		std::cout << "Actuator 6   current : " << current.Actuators.Actuator6 << " A" << std::endl;
		std::cout << "Actuator 7   current : " << current.Actuators.Actuator7 << " A" << std::endl;
		std::cout << "*********************************" << std::endl << std::endl << std::endl;
	}

	std::vector<float> currents;
	currents.push_back(current.Actuators.Actuator1);
	currents.push_back(current.Actuators.Actuator2);
	currents.push_back(current.Actuators.Actuator3);
	currents.push_back(current.Actuators.Actuator4);
	currents.push_back(current.Actuators.Actuator5);
	currents.push_back(current.Actuators.Actuator6);
	currents.push_back(current.Actuators.Actuator7);
	return currents;
}

std::vector<float> get_angular_info(int actuator_id, int result)
{
	AngularPosition dataCommand;
	AngularPosition dataPosition;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		(*MyGetAngularCommand)(dataCommand);
		(*MyGetAngularPosition)(dataPosition);

		std::cout << "*********************************" << std::endl;
		std::cout << "Actuator 1   command : " << dataCommand.Actuators.Actuator1 << " deg" << "     Position : " << dataPosition.Actuators.Actuator1 << " deg" << std::endl;
		std::cout << "Actuator 2   command : " << dataCommand.Actuators.Actuator2 << " deg" << "     Position : " << dataPosition.Actuators.Actuator2 << " deg" << std::endl;
		std::cout << "Actuator 3   command : " << dataCommand.Actuators.Actuator3 << " deg" << "     Position : " << dataPosition.Actuators.Actuator3 << " deg" << std::endl;
		std::cout << "Actuator 4   command : " << dataCommand.Actuators.Actuator4 << " deg" << "     Position : " << dataPosition.Actuators.Actuator4 << " deg" << std::endl;
		std::cout << "Actuator 5   command : " << dataCommand.Actuators.Actuator5 << " deg" << "     Position : " << dataPosition.Actuators.Actuator5 << " deg" << std::endl;
		std::cout << "Actuator 6   command : " << dataCommand.Actuators.Actuator6 << " deg" << "     Position : " << dataPosition.Actuators.Actuator6 << " deg" << std::endl;
		std::cout << "Actuator 7   command : " << dataCommand.Actuators.Actuator7 << " deg" << "     Position : " << dataPosition.Actuators.Actuator7 << " deg" << std::endl << std::endl;

		std::cout << "  Finger 1   command: " << dataCommand.Fingers.Finger1 << "     Position : " << dataPosition.Fingers.Finger1 << std::endl;
		std::cout << "  Finger 2   command: " << dataCommand.Fingers.Finger2 << "     Position : " << dataPosition.Fingers.Finger2 << std::endl;
		std::cout << "  Finger 3   command: " << dataCommand.Fingers.Finger3 << "     Position : " << dataPosition.Fingers.Finger3 << std::endl;
		std::cout << "*********************************" << std::endl << std::endl << std::endl;
	}

	std::vector<float> response;

	switch (actuator_id)
	{
		case 1:
			response.push_back(dataCommand.Actuators.Actuator1);
			response.push_back(dataPosition.Actuators.Actuator1);
			break;
		case 2:
			response.push_back(dataCommand.Actuators.Actuator2);
			response.push_back(dataPosition.Actuators.Actuator2);
			break;
		case 3:
			response.push_back(dataCommand.Actuators.Actuator3);
			response.push_back(dataPosition.Actuators.Actuator3);
			break;
		case 4:
			response.push_back(dataCommand.Actuators.Actuator4);
			response.push_back(dataPosition.Actuators.Actuator4);
			break;
		case 5:
			response.push_back(dataCommand.Actuators.Actuator5);
			response.push_back(dataPosition.Actuators.Actuator5);
			break;
		case 6:
			response.push_back(dataCommand.Actuators.Actuator6);
			response.push_back(dataPosition.Actuators.Actuator6);
			break;
		case 7:
			response.push_back(dataCommand.Actuators.Actuator2);
			response.push_back(dataPosition.Actuators.Actuator2);
			break;
		default:
			response.push_back(-9999);
			response.push_back(-9999);
			break;
	}

	return response;
}

std::vector<std::vector<float>> get_angular_infos(int result)
{
	AngularPosition dataCommand;
	AngularPosition dataPosition;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		(*MyGetAngularCommand)(dataCommand);
		(*MyGetAngularPosition)(dataPosition);

		std::cout << "*********************************" << std::endl;
		std::cout << "Actuator 1   command : " << dataCommand.Actuators.Actuator1 << " deg" << "     Position : " << dataPosition.Actuators.Actuator1 << " deg" << std::endl;
		std::cout << "Actuator 2   command : " << dataCommand.Actuators.Actuator2 << " deg" << "     Position : " << dataPosition.Actuators.Actuator2 << " deg" << std::endl;
		std::cout << "Actuator 3   command : " << dataCommand.Actuators.Actuator3 << " deg" << "     Position : " << dataPosition.Actuators.Actuator3 << " deg" << std::endl;
		std::cout << "Actuator 4   command : " << dataCommand.Actuators.Actuator4 << " deg" << "     Position : " << dataPosition.Actuators.Actuator4 << " deg" << std::endl;
		std::cout << "Actuator 5   command : " << dataCommand.Actuators.Actuator5 << " deg" << "     Position : " << dataPosition.Actuators.Actuator5 << " deg" << std::endl;
		std::cout << "Actuator 6   command : " << dataCommand.Actuators.Actuator6 << " deg" << "     Position : " << dataPosition.Actuators.Actuator6 << " deg" << std::endl;
		std::cout << "Actuator 7   command : " << dataCommand.Actuators.Actuator7 << " deg" << "     Position : " << dataPosition.Actuators.Actuator7 << " deg" << std::endl << std::endl;

		std::cout << "  Finger 1   command: " << dataCommand.Fingers.Finger1 << "     Position : " << dataPosition.Fingers.Finger1 << std::endl;
		std::cout << "  Finger 2   command: " << dataCommand.Fingers.Finger2 << "     Position : " << dataPosition.Fingers.Finger2 << std::endl;
		std::cout << "  Finger 3   command: " << dataCommand.Fingers.Finger3 << "     Position : " << dataPosition.Fingers.Finger3 << std::endl;
		std::cout << "*********************************" << std::endl << std::endl << std::endl;

		std::vector<std::vector<float>> angular_infos;
		std::vector<float> dataCmd;
		std::vector<float> dataPos;

		dataCmd.push_back(dataCommand.Actuators.Actuator1);
		dataCmd.push_back(dataCommand.Actuators.Actuator2);
		dataCmd.push_back(dataCommand.Actuators.Actuator3);
		dataCmd.push_back(dataCommand.Actuators.Actuator4);
		dataCmd.push_back(dataCommand.Actuators.Actuator5);
		dataCmd.push_back(dataCommand.Actuators.Actuator6);
		dataCmd.push_back(dataCommand.Actuators.Actuator7);

		dataPos.push_back(dataPosition.Actuators.Actuator1);
		dataPos.push_back(dataPosition.Actuators.Actuator2);
		dataPos.push_back(dataPosition.Actuators.Actuator3);
		dataPos.push_back(dataPosition.Actuators.Actuator4);
		dataPos.push_back(dataPosition.Actuators.Actuator5);
		dataPos.push_back(dataPosition.Actuators.Actuator6);
		dataPos.push_back(dataPosition.Actuators.Actuator7);

		angular_infos.push_back(dataCmd);
		angular_infos.push_back(dataPos);
	}
	std::vector<std::vector<float>> null;
	return null;
}

std::vector<CartesianInfo> get_cartesian_info(int actuator_id, int result)
{
	CartesianPosition dataCommand;
	CartesianPosition dataPosition;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		(*MyGetCartesianCommand)(dataCommand);
		(*MyGetCartesianPosition)(dataPosition);

		std::cout << "*********************************" << std::endl;
		std::cout << "X   command : " << dataCommand.Coordinates.X << " m" << "     Position : " << dataPosition.Coordinates.X << " m" << std::endl;
		std::cout << "Y   command : " << dataCommand.Coordinates.Y << " m" << "     Position : " << dataPosition.Coordinates.Y << " m" << std::endl;
		std::cout << "Z   command : " << dataCommand.Coordinates.Z << " m" << "     Position : " << dataPosition.Coordinates.Z << " m" << std::endl;
		std::cout << "Theta X   command : " << dataCommand.Coordinates.ThetaX << " Rad" << "     Position : " << dataPosition.Coordinates.ThetaX << " Rad" << std::endl;
		std::cout << "Theta Y   command : " << dataCommand.Coordinates.ThetaY << " Rad" << "     Position : " << dataPosition.Coordinates.ThetaY << " Rad" << std::endl;
		std::cout << "Theta Z   command : " << dataCommand.Coordinates.ThetaZ << " Rad" << "     Position : " << dataPosition.Coordinates.ThetaZ << " Rad" << std::endl << std::endl;

		std::cout << "  Finger 1   command: " << dataCommand.Fingers.Finger1 << "     Position : " << dataPosition.Fingers.Finger1 << std::endl;
		std::cout << "  Finger 2   command: " << dataCommand.Fingers.Finger2 << "     Position : " << dataPosition.Fingers.Finger2 << std::endl;
		std::cout << "  Finger 3   command: " << dataCommand.Fingers.Finger3 << "     Position : " << dataPosition.Fingers.Finger3 << std::endl;
		std::cout << "*********************************" << std::endl << std::endl << std::endl;
		
		std::vector<CartesianInfo> response;
		response.push_back(dataCommand.Coordinates);
		response.push_back(dataPosition.Coordinates);
		return response;
	}
	std::vector<CartesianInfo> null;
	return null;
}

float get_temperature(int actuator_id, int result)
{
	if (actuator_id > 7) return -9999;

	GeneralInformations data;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		MyGetGeneralInformations(data);

		std::cout << "*********************************" << std::endl;
		std::cout << "Actuator 1 temperature : " << data.ActuatorsTemperatures[0] << " °C" << std::endl;
		std::cout << "Actuator 2 temperature : " << data.ActuatorsTemperatures[1] << " °C" << std::endl;
		std::cout << "Actuator 3 temperature : " << data.ActuatorsTemperatures[2] << " °C" << std::endl;
		std::cout << "Actuator 4 temperature : " << data.ActuatorsTemperatures[3] << " °C" << std::endl;
		std::cout << "Actuator 5 temperature : " << data.ActuatorsTemperatures[4] << " °C" << std::endl;
		std::cout << "Actuator 6 temperature : " << data.ActuatorsTemperatures[5] << " °C" << std::endl;
		std::cout << "Actuator 7 temperature : " << data.ActuatorsTemperatures[6] << " °C" << std::endl;
		std::cout << "*********************************" << std::endl << std::endl << std::endl;
	}

	return data.ActuatorsTemperatures[actuator_id - 1];
}

std::vector<float> get_temperatures(int result)
{
	
	GeneralInformations data;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		MyGetGeneralInformations(data);

		std::cout << "*********************************" << std::endl;
		std::cout << "Actuator 1 temperature : " << data.ActuatorsTemperatures[0] << " °C" << std::endl;
		std::cout << "Actuator 2 temperature : " << data.ActuatorsTemperatures[1] << " °C" << std::endl;
		std::cout << "Actuator 3 temperature : " << data.ActuatorsTemperatures[2] << " °C" << std::endl;
		std::cout << "Actuator 4 temperature : " << data.ActuatorsTemperatures[3] << " °C" << std::endl;
		std::cout << "Actuator 5 temperature : " << data.ActuatorsTemperatures[4] << " °C" << std::endl;
		std::cout << "Actuator 6 temperature : " << data.ActuatorsTemperatures[5] << " °C" << std::endl;
		std::cout << "Actuator 7 temperature : " << data.ActuatorsTemperatures[6] << " °C" << std::endl;
		std::cout << "*********************************" << std::endl << std::endl << std::endl;
	}

	std::vector<float> response;
	response.push_back(data.ActuatorsTemperatures[0]);
	response.push_back(data.ActuatorsTemperatures[1]);
	response.push_back(data.ActuatorsTemperatures[2]);
	response.push_back(data.ActuatorsTemperatures[3]);
	response.push_back(data.ActuatorsTemperatures[4]);
	response.push_back(data.ActuatorsTemperatures[5]);
	response.push_back(data.ActuatorsTemperatures[6]);
	return response;
}

float get_torque_value(int actuator_id, int result, int modifier)
{
	if (actuator_id > 7) return -9999;

	float acts[7];
	float gfacts[7];

	AngularPosition torque;
	AngularPosition torqueGravityFree;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;


		acts[0] = torque.Actuators.Actuator1;
		acts[1] = torque.Actuators.Actuator2;
		acts[2] = torque.Actuators.Actuator3;
		acts[3] = torque.Actuators.Actuator4;
		acts[4] = torque.Actuators.Actuator5;
		acts[5] = torque.Actuators.Actuator6;
		acts[6] = torque.Actuators.Actuator7;
		
		
		gfacts[0] = torqueGravityFree.Actuators.Actuator1;
		gfacts[1] = torqueGravityFree.Actuators.Actuator2;
		gfacts[2] = torqueGravityFree.Actuators.Actuator3;
		gfacts[3] = torqueGravityFree.Actuators.Actuator4;
		gfacts[4] = torqueGravityFree.Actuators.Actuator5;
		gfacts[5] = torqueGravityFree.Actuators.Actuator6;
		gfacts[6] = torqueGravityFree.Actuators.Actuator7;
		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		MyGetAngularForce(torque);
		MyGetAngularForceGravityFree(torqueGravityFree);
		std::cout << "*********************************" << std::endl;
		std::cout << "Actuator 1   torque : " << acts[0] << " N*m" << "     without gravity : " << gfacts[0] << " N*m" << std::endl;
		std::cout << "Actuator 2   torque : " << acts[1] << " N*m" << "     without gravity : " << gfacts[1] << " N*m" << std::endl;
		std::cout << "Actuator 3   torque : " << acts[2] << " N*m" << "     without gravity : " << gfacts[2] << " N*m" << std::endl;
		std::cout << "Actuator 4   torque : " << acts[3] << " N*m" << "     without gravity : " << gfacts[3] << " N*m" << std::endl;
		std::cout << "Actuator 5   torque : " << acts[4] << " N*m" << "     without gravity : " << gfacts[4] << " N*m" << std::endl;
		std::cout << "Actuator 6   torque : " << acts[5] << " N*m" << "     without gravity : " << gfacts[5] << " N*m" << std::endl;
		std::cout << "Actuator 7   torque : " << acts[6] << " N*m" << "     without gravity : " << gfacts[6] << " N*m" << std::endl;
		std::cout << "*********************************" << std::endl << std::endl << std::endl;
	}

	if (modifier == 0)
	{
		return acts[actuator_id - 1];
	} else if (modifier == 1)
	{
		return gfacts[actuator_id - 1];
	} else  {
		return -9999;
	}
	return -9999;
}

std::vector<float> get_torque_values(int result, int modifier)
{

	float acts[7];
	float gfacts[7];

	AngularPosition torque;
	AngularPosition torqueGravityFree;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;


		acts[0] = torque.Actuators.Actuator1;
		acts[1] = torque.Actuators.Actuator2;
		acts[2] = torque.Actuators.Actuator3;
		acts[3] = torque.Actuators.Actuator4;
		acts[4] = torque.Actuators.Actuator5;
		acts[5] = torque.Actuators.Actuator6;
		acts[6] = torque.Actuators.Actuator7;
		
		
		gfacts[0] = torqueGravityFree.Actuators.Actuator1;
		gfacts[1] = torqueGravityFree.Actuators.Actuator2;
		gfacts[2] = torqueGravityFree.Actuators.Actuator3;
		gfacts[3] = torqueGravityFree.Actuators.Actuator4;
		gfacts[4] = torqueGravityFree.Actuators.Actuator5;
		gfacts[5] = torqueGravityFree.Actuators.Actuator6;
		gfacts[6] = torqueGravityFree.Actuators.Actuator7;
		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		MyGetAngularForce(torque);
		MyGetAngularForceGravityFree(torqueGravityFree);
		std::cout << "*********************************" << std::endl;
		std::cout << "Actuator 1   torque : " << acts[0] << " N*m" << "     without gravity : " << gfacts[0] << " N*m" << std::endl;
		std::cout << "Actuator 2   torque : " << acts[1] << " N*m" << "     without gravity : " << gfacts[1] << " N*m" << std::endl;
		std::cout << "Actuator 3   torque : " << acts[2] << " N*m" << "     without gravity : " << gfacts[2] << " N*m" << std::endl;
		std::cout << "Actuator 4   torque : " << acts[3] << " N*m" << "     without gravity : " << gfacts[3] << " N*m" << std::endl;
		std::cout << "Actuator 5   torque : " << acts[4] << " N*m" << "     without gravity : " << gfacts[4] << " N*m" << std::endl;
		std::cout << "Actuator 6   torque : " << acts[5] << " N*m" << "     without gravity : " << gfacts[5] << " N*m" << std::endl;
		std::cout << "Actuator 7   torque : " << acts[6] << " N*m" << "     without gravity : " << gfacts[6] << " N*m" << std::endl;
		std::cout << "*********************************" << std::endl << std::endl << std::endl;
	}

	std::vector<float> response;

	if (modifier == 0)
	{
		response.push_back(acts[0]);
		response.push_back(acts[1]);
		response.push_back(acts[2]);
		response.push_back(acts[3]);
		response.push_back(acts[4]);
		response.push_back(acts[5]);
		response.push_back(acts[6]);

	} else if (modifier == 1)
	{
		response.push_back(gfacts[0]);
		response.push_back(gfacts[1]);
		response.push_back(gfacts[2]);
		response.push_back(gfacts[3]);
		response.push_back(gfacts[4]);
		response.push_back(gfacts[5]);
		response.push_back(gfacts[6]);
	} else  {
		return response;
	}

	return response;
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
		std::cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << std::endl;
		programResult = 0;
	}
	else
	{
		std::cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << std::endl << std::endl;
		programResult = 1;

	}
    return programResult;
}

// ZEROMQ API functions ____ SECTION 2 OF CODE

int communication_bridge(int result);
void sleep(int duration);
std::vector<std::string> splitstr(std::string str, char seperator);
int timestamp();

int main(int argc, char *argv[])
{
    int is_initialized = InitializeAPIFunctions();
    if (!is_initialized) return EXIT_FAILURE;

    int result = (*MyInitAPI)();
    int programResult = communication_bridge(result);
    return programResult;
}

int communication_bridge(int result) {
    // Create a ZeroMQ context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_REP);

    // Bind the socket to a TCP address (we use localhost and port 5555)
    socket.bind("tcp://*:5555"); // tcp://*:5555/ would be accepted
                                         // as well

    
    std::cout << "ZeroMQ reply server is running..." << std::endl;

    while (true) {
        // Wait for a request from a client
        zmq::message_t request;
        socket.recv(&request);
        // Process the request (in this example, we simply echo back the message)
        std::string receivedMessage(static_cast<char*>(request.data()), request.size());
        std::cout << "Received request: " << receivedMessage << std::endl;

        std::vector<std::string> request_payload = splitstr(receivedMessage,';');
        std::string replyMessage;

        if (request_payload[0] == "get_motor_by_id" && stoi(request_payload[1]) <= 6) {
            int motor_id = stoi(request_payload[1]);
			replyMessage = "You motor_id: " + std::to_string(motor_id);
            // Send the reply back to the client
        } else if (request_payload[0] == "get_current" && stoi(request_payload[1]) <= 6)
        {
			replyMessage = "[0,0,0,0,0,0]";
        } else if (request_payload[0] == "get_temperature" && stoi(request_payload[1]) <= 6)
        {
            
        } else if (request_payload[0] == "get_angular_info" && stoi(request_payload[1]) <= 6)
        {
            
        } else if (request_payload[0] == "get_cartesian_info" && stoi(request_payload[1]) <= 6) 
        {

        } else if (request_payload[0] == "get_torque_value" && stoi(request_payload[1]) <= 6) 
        {

        } else if(request_payload[0] == "get_temperatures") 
        {

        } else if(request_payload[0] == "get_angular_infos") 
        {

        } else if(request_payload[0] == "get_cartesian_infos") 
        {

        } else if(request_payload[0] == "get_torque_values") 
        {

        } else if(request_payload[0] == "get_torque_values") 
        {

        } else if(request_payload[0] == "get_torque_values") 
        {

        }  else {
            std::cout << "Error: Requst is not sending clearly" << std::endl;
            std::cout << "Request payload: " << receivedMessage << std::endl;
        }
        zmq::message_t reply(replyMessage.size());
        memcpy(reply.data(), replyMessage.data(), replyMessage.size());
        socket.send(reply, zmq::send_flags::none);
        std::cout << "Sent reply: " << replyMessage << std::endl;
    }

    // The server should never reach this point, but for completeness, close the socket and context
    socket.close();
    context.close();

    return EXIT_SUCCESS;
}


int timestamp() {
    return std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch())
        .count();
}

// Create custom split() function.  
std::vector<std::string> splitstr(std::string str, char separator) {
    std::vector<std::string> vString;
    int startIndex = 0, endIndex = 0;
    for (int i = 0; i <= str.size(); i++) {
        
        // If we reached the end of the word or the end of the input.
        if (str[i] == separator || i == str.size()) {
            endIndex = i;
            std::string temp;
            temp.append(str, startIndex, endIndex - startIndex);
            vString.push_back(temp);
            startIndex = endIndex + 1;
        }
    }

    return vString;
}