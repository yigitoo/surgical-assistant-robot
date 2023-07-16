#ifndef __API_H__
#define __API_H__
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
int admittance_control(int actuator_id, int result);
int angular_control(int actuator_id, int result);
int cartesian_control(int actuator_id, int result);
int get_actuator_current(int actuator_id, int result);
int get_angular_info(int actuator_id, int result);
int get_cartesian_info(int actuator_id, int result);
int get_temperature(int actuator_id, int result);
int get_torque_value(int actuator_id, int result);
int InitializeAPIFunctions();

#endif // __API_H__