#include <iostream>
#include <string>
#include <vector>
#include <chrono>

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

#define PI 3.14159265358979323846

#include <zmq.hpp>

// KINOVA K-75+ & K-58 ACTUATORS API functions ____ SECTION 1 OF CODE
#ifdef __linux__ 
void * commandLayer_handle;
#elif _WIN32
HINSTANCE commandLayer_handle;
#endif

void sleep(int duration) {
#ifdef __linux__ 
	usleep(duration * 1000);
#elif _WIN32
	Sleep(duration);	
#endif
}

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
void angular_control(std::vector<float> fv_angularVelocity, int result);
std::vector<float> goto_home(int result, int init) {
	if (init == 1)
	{
		std::vector<float> degrees{0,210,210,0,120,0};
		angular_control(degrees, result);
		return degrees;
	}
	//Setting the current device as the active device.
	std::vector<float> degrees{0,179.2,115.7,0,128.56,0};
	angular_control(degrees, result);
	return degrees;
}

int admittance_control(int result)
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

void angular_control(std::vector<float> fv_angles, int result)
{
	AngularPosition currentCommand;
	TrajectoryPoint pointToSend;
	pointToSend.InitStruct();

	//We specify that this point will be used an angular(joint by joint) velocity vector.
	pointToSend.Position.Type = ANGULAR_POSITION;

	pointToSend.Position.Actuators.Actuator1 = fv_angles[0];
	pointToSend.Position.Actuators.Actuator2 = fv_angles[1];
	pointToSend.Position.Actuators.Actuator3 = fv_angles[2];
	pointToSend.Position.Actuators.Actuator4 = fv_angles[3];
	pointToSend.Position.Actuators.Actuator5 = fv_angles[4];
	pointToSend.Position.Actuators.Actuator6 = fv_angles[5];
	
	//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
	MySendBasicTrajectory(pointToSend);

}

void cartesian_control(std::vector<float> fv_cartesianVelocity, int result)
{
    CartesianPosition currentCommand;
	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		std::cout << "Initializing the fingers" << std::endl;
		MyInitFingers();

		TrajectoryPoint pointToSend;
		pointToSend.InitStruct();

		//We specify that this point will be used an angular(joint by joint) velocity vector.
		pointToSend.Position.Type = CARTESIAN_POSITION;

		pointToSend.Position.CartesianPosition.X = fv_cartesianVelocity[0];
		pointToSend.Position.CartesianPosition.Y = fv_cartesianVelocity[1];
		pointToSend.Position.CartesianPosition.Z = fv_cartesianVelocity[2];
		pointToSend.Position.CartesianPosition.ThetaX = fv_cartesianVelocity[3];
		pointToSend.Position.CartesianPosition.ThetaY = fv_cartesianVelocity[4];
		pointToSend.Position.CartesianPosition.ThetaZ = fv_cartesianVelocity[5];

		pointToSend.Position.Fingers.Finger1 = 0;
		pointToSend.Position.Fingers.Finger2 = 0;
		pointToSend.Position.Fingers.Finger3 = 0;

		//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
		MySendBasicTrajectory(pointToSend);
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

std::vector<std::vector<float>> get_angular_infos(int result)
{
	AngularPosition dataCommand;
	AngularPosition dataPosition;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		(*MyGetAngularCommand)(dataCommand);
		(*MyGetAngularPosition)(dataPosition);

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

std::vector<CartesianInfo> get_cartesian_infos(int result)
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

		
		std::vector<CartesianInfo> response;
		response.push_back(dataCommand.Coordinates);
		response.push_back(dataPosition.Coordinates);
		return response;
	}
	std::vector<CartesianInfo> null;
	return null;
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

std::vector<float> get_torque_values(int result)
{
	AngularPosition torque;
	AngularPosition torqueGravityFree;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;

		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		MyGetAngularForce(torque);
		MyGetAngularForceGravityFree(torqueGravityFree);
	}
	
	std::vector<float> response;
	response.push_back(torque.Actuators.Actuator1);
	response.push_back(torque.Actuators.Actuator2);
	response.push_back(torque.Actuators.Actuator3);
	response.push_back(torque.Actuators.Actuator4);
	response.push_back(torque.Actuators.Actuator5);
	response.push_back(torque.Actuators.Actuator6);
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

int communication_bridge(int result, std::vector<float> current_degrees);
void sleep(int duration);
std::vector<std::string> splitstr(std::string str, char seperator);
std::string float_vector_to_string(std::vector<float> vector);
std::vector<float> string_to_float_vector(std::string vector_string);
std::vector<float> make_cartesian_to_float_vector(CartesianInfo cartesian_info);
std::vector<float> degrees_to_radians(std::vector<float> degrees);
std::vector<float> radians_to_degrees(std::vector<float> radians);

int main(int argc, char *argv[])
{
	std::vector<float> current_degrees, arguments;
	for(int i = 1; i < argc; i++)
	{
		arguments.push_back(std::atof(argv[i]));
	}
			
    int is_initialized = InitializeAPIFunctions();
    if (!is_initialized) return EXIT_FAILURE;

    int result = (*MyInitAPI)();
	if (result == ERROR_NO_DEVICE_FOUND)
	{
		std::cout << "DEVICE NOT FOUND ON USB SOCKET" << std::endl;
		return EXIT_FAILURE;
	}

	KinovaDevice list[MAX_KINOVA_DEVICE];
	int devicesCount = MyGetDevices(list, result);

	for (int i = 0; i < devicesCount; i++)
	{
		//Setting the current device as the active device.
		MySetActiveDevice(list[i]);
	}
	current_degrees = goto_home(result, 1);
	if (argc > 1) angular_control(arguments, result);

    int programResult = communication_bridge(result, current_degrees);
	result = (*MyCloseAPI)();
    return programResult;
}

std::vector<float> degrees_to_radians(std::vector<float> degrees)
{
	std::vector<float> radians;
	radians.push_back(degrees[0] * (PI / 180));
	radians.push_back(degrees[1] * (PI / 180));
	radians.push_back(degrees[2] * (PI / 180));
	radians.push_back(degrees[3] * (PI / 180));
	radians.push_back(degrees[4] * (PI / 180));
	radians.push_back(degrees[5] * (PI / 180));
	return radians;
}

std::vector<float> radians_to_degrees(std::vector<float> radians)
{
	std::vector<float> degrees;
    degrees.push_back(radians[0] * (180 / PI));
    degrees.push_back(radians[1] * (180 / PI));
    degrees.push_back(radians[2] * (180 / PI));
    degrees.push_back(radians[3] * (180 / PI));
    degrees.push_back(radians[4] * (180 / PI));
    degrees.push_back(radians[5] * (180 / PI));
    return degrees;
}

std::string float_vector_to_string(std::vector<float> vector)
{
	std::string result;
	for(int i = 0; i < vector.size() - 1; i++)
	{
		result += std::to_string(vector[i]) + ",";
	}

	result += std::to_string(vector[vector.size() - 1]);
	return result;
}

std::vector<float> string_to_float_vector(std::string vector_string)
{
	std::vector<float> result;
    std::vector<std::string> split_vector = splitstr(vector_string, ',');
    for(int i = 0; i < split_vector.size(); i++)
    {
        result.push_back(std::stof(split_vector[i]));
    }
    return result;
}

std::vector<float> make_cartesian_to_float_vector(CartesianInfo cartesian_info)
{
	std::vector<float> result;
	result.push_back(cartesian_info.X);
	result.push_back(cartesian_info.Y);
	result.push_back(cartesian_info.Z);
	result.push_back(cartesian_info.ThetaX);
	result.push_back(cartesian_info.ThetaY);
	result.push_back(cartesian_info.ThetaZ);
	return result;
}

int communication_bridge(int result, std::vector<float> current_degrees) {
    // Create a ZeroMQ context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_REP);

    // Bind the socket to a TCP address (we use localhost and port 5555)
    socket.bind("tcp://*:456"); // tcp://*:5555/ would be accepted
                                         // as well

    
    std::cout << "ZeroMQ reply server is running..." << std::endl;

    while (1)
	{
        // Wait for a request from a client
        zmq::message_t request;
        socket.recv(&request);
        // Process the request (in this example, we simply echo back the message)
        std::string receivedMessage(static_cast<char*>(request.data()), request.size());
        std::cout << "Received request: " << receivedMessage << std::endl;

        std::vector<std::string> request_payload = splitstr(receivedMessage,';');
        std::string replyMessage;
		if (request_payload[0] == "get_all")
		{
			// CURRENT
			std::vector<float> currents = get_actuator_currents(result);
            replyMessage = float_vector_to_string(currents);

			replyMessage += ":";

			// TEMPERATURE
			std::vector<float> temperatures = get_temperatures(result);
			replyMessage += float_vector_to_string(temperatures);

			replyMessage += ":";


			// ANGULAR_VEL INFO
			std::string temp;
			std::vector<std::vector<float>> angles = get_angular_infos(result);
			temp = float_vector_to_string(angles[0]);
			replyMessage += temp + ";";
			temp += float_vector_to_string(angles[1]);
			replyMessage += temp;

			replyMessage += ":";

			// CARTESIAN_VEL INFO
			std::vector<CartesianInfo> cartesian = get_cartesian_infos(result);
			replyMessage += float_vector_to_string(make_cartesian_to_float_vector(cartesian[0]));
			replyMessage += ";";
			replyMessage += float_vector_to_string(make_cartesian_to_float_vector(cartesian[1]));

			replyMessage += ":";

			// TORQUE VALUES
			std::vector<float> torque_values = get_torque_values(result);
			replyMessage += float_vector_to_string(torque_values);
   
		} else if (request_payload[0] == "set_angle")
		{
			std::vector<float> angular_velocity = string_to_float_vector(request_payload[1]);
			if (angular_velocity.size() != 6) throw std::runtime_error("YOU CANNOT SEND A VECTOR THAT HAVE NOT 6 LENGTH");  

			angular_control(angular_velocity, result);

		} else if (request_payload[0] == "inc_angle")
		{
			if(current_degrees.size() != 6) throw std::runtime_error("YOU CANNOT SEND A VECTOR THAT HAVE NOT 6 LENGTH");
			std::vector<float> degrees = string_to_float_vector(request_payload[1]);
			current_degrees[0] += degrees[0];
			current_degrees[1] += degrees[1];
			current_degrees[2] += degrees[2];
			current_degrees[3] += degrees[3];
			current_degrees[4] += degrees[4];
			current_degrees[5] += degrees[5];
			angular_control(current_degrees, result);
		} else if(request_payload[0] == "set_cartesian")
		{
			std::vector<float> cartesian_velocity = string_to_float_vector(request_payload[1]);
			if (cartesian_velocity.size() != 6) throw std::runtime_error("YOU CANNOT SEND A VECTOR THAT HAVE NOT 6 LENGTH");  

			cartesian_control(cartesian_velocity, result);

		} else if (request_payload[0] == "goto_home")
		{
			goto_home(result, 0);
			replyMessage = "The robot has been successfully positioned on the initial home.";
		} else if (request_payload[0] == "goto_home2") 
		{
			goto_home(result, 1);
			replyMessage = "The robot has been successfully positioned on the initial home.";

		} else if (request_payload[0] == "get_current")
        {
			std::vector<float> currents = get_actuator_currents(result);
            replyMessage = float_vector_to_string(currents);
        } else if (request_payload[0] == "get_temperature")
        {
			std::vector<float> temperatures = get_temperatures(result);
			replyMessage = float_vector_to_string(temperatures);
        } else if (request_payload[0] == "get_angular_info")
        {
			std::string temp;
			std::vector<std::vector<float>> angles = get_angular_infos(result);
			
			temp = float_vector_to_string(angles[0]);
			replyMessage = temp + ";";
			
			temp = float_vector_to_string(angles[1]);
			replyMessage += temp;

        } else if (request_payload[0] == "get_cartesian_info") 
        {
			std::vector<CartesianInfo> cartesian = get_cartesian_infos(result);
			replyMessage = float_vector_to_string(make_cartesian_to_float_vector(cartesian[0]));
			replyMessage += ";";
			replyMessage += float_vector_to_string(make_cartesian_to_float_vector(cartesian[1]));
				
        } else if (request_payload[0] == "get_torque_value") 
        {
			std::vector<float> torque_values = get_torque_values(result);
			replyMessage = float_vector_to_string(torque_values);
        } else {
            std::cout << "Error: Requst is not sending clearly" << std::endl;
            std::cout << "Request payload: " << receivedMessage << std::endl;
			continue;
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