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

void sleep(int duration);
int admittance_control(int actuator_id, int result);

std::vector<float> angular_control(int actuator_id, int result);

CartesianInfo cartesian_control(int result);

float get_actuator_current(int actuator_id, int result);
std::vector<float> get_actuator_currents(int result);

std::vector<float> get_angular_info(int actuator_id, int result);
std::vector<std::vector<float>> get_angular_infos(int result);

std::vector<CartesianInfo> get_cartesian_info(int actuator_id, int result);

float get_temperature(int actuator_id, int result);
std::vector<float> get_temperatures(int result);

float get_torque_value(int actuator_id, int result, int modifier);
std::vector<float> get_torque_values(int result);

int InitializeAPIFunctions();

#endif // __API_H__