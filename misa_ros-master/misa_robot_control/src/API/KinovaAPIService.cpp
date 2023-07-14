#include "ros/ros.h"
#include <signal.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

#include "misa_files/KinovaCommandMessage.h"
#include "misa_files/KinovaSensorService.h"
#include "misa_files/KinovaStatusService.h"

#include "misa_files/KinovaErrorMessage.h"
#include "misa_files/KinovaInfoMessage.h"

#include "KinovaErrors.h"
#include <cmath>

#include <sstream>
#include <iostream>
#include <dlfcn.h>
#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>

using namespace std;

int result;

// Control Variables
//Basic angular control variables
TrajectoryPoint pointToSend;
//Torque Control variables
TORQUECONTROL_TYPE torqueControlType = DIRECTTORQUE;
GENERALCONTROL_TYPE generalControlType = TORQUE;

AngularPosition DataCommand;

string currentControlType;
//Offset Variables:
float angleOffsets[4];
float TorqueCommand[COMMAND_SIZE];
float TorqueFIFO[COMMAND_SIZE];

misa_files::KinovaErrorMessage errorMessage;
misa_files::KinovaInfoMessage infoMessage;

std::vector<float> torqueArray;

bool initial;
bool enableTorqueCloseLoop;
bool emergencyMode, isFunctional;
int rosFrequency;
//Sensor Variables:

AngularPosition angleData;
AngularPosition currentData;
AngularPosition torqueData;
AngularPosition velocityData;
AngularAcceleration accelerationData;
SensorsInfo temperatureData;

ros::Publisher errorPub;
ros::Publisher infoPub;

void * commandLayer_handle = commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
//Function pointers to the functions we need
int(*MyInitAPI)() = (int(*)()) dlsym(commandLayer_handle, "InitAPI");
int(*MyCloseAPI)() = (int(*)()) dlsym(commandLayer_handle, "CloseAPI");

int(*MyGetAngularCommand)(AngularPosition &) = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
int(*MyGetAngularPosition)(AngularPosition &) = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result) = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
int(*MySetActiveDevice)(KinovaDevice device) = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");

int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]) = (int(*)(ROBOT_TYPE, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) dlsym(commandLayer_handle, "RunGravityZEstimationSequence");
int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE) = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
int(*MySetTorqueSafetyFactor)(float factor) = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
int(*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]) = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendAngularTorqueCommand");
int(*MySendBasicTrajectory)(TrajectoryPoint command) =	(int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
int(*MySetGravityVector)(float Command[3]) = (int(*)(float Command[3])) dlsym(commandLayer_handle, "SetGravityVector");
int(*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]) = (int(*)(float Command[GRAVITY_PAYLOAD_SIZE])) dlsym(commandLayer_handle, "SetGravityPayload");
int(*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]) = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "SetGravityOptimalZParam");
int(*MySetGravityType)(GRAVITY_TYPE Type) = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");
int(*MySetTorqueVibrationController)(float value) = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueVibrationController");
int(*MyGetAngularForceGravityFree)(AngularPosition &) = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularForceGravityFree");;
int(*MySetTorqueControlType)(TORQUECONTROL_TYPE type) = (int(*)(TORQUECONTROL_TYPE)) dlsym(commandLayer_handle, "SetTorqueControlType");

int (*MyGetAngularCurrentMotor)(AngularPosition &) = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCurrentMotor");
int (*MyGetAngularForce)(AngularPosition &) = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularForce");
int (*MyGetTemperature)(SensorsInfo &) = (int (*)(SensorsInfo &)) dlsym(commandLayer_handle,"GetSensorsInfo");
int (*MyGetActuatorAcceleration)(AngularAcceleration &) = (int (*)(AngularAcceleration &)) dlsym(commandLayer_handle,"GetActuatorAcceleration");
int (*MyGetAngularVelocity)(AngularPosition &) = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularVelocity");
int (*MySetTorqueZero)(int) = (int (*)(int)) dlsym(commandLayer_handle,"SetTorqueZero");

int (*MyGetTrajectoryTorqueMode)(int &) = (int(*)(int &)) dlsym(commandLayer_handle, "GetTrajectoryTorqueMode");
int (*MyGetActualTrajectoryInfo)(TrajectoryPoint &) = (int (*)(TrajectoryPoint &)) dlsym(commandLayer_handle,"GetActualTrajectoryInfo");
int (*MyGetAngularTorqueCommand)(float Command[GRAVITY_PARAM_SIZE]) = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "GetAngularTorqueCommand");


void reportError(string source, string error, int priority)
{
    errorMessage.header.stamp = ros::Time::now();
    errorMessage.source = source;
    errorMessage.error = error;
    errorMessage.priority = priority;
    errorPub.publish(errorMessage);
}

void reportInfo(string source, string reason, bool functional, bool emergency)
{
    infoMessage.header.stamp = ros::Time::now();
    infoMessage.source = source;
    infoMessage.reason = reason;
    infoMessage.functional = functional;
    infoMessage.emergency = emergency;
    infoPub.publish(infoMessage);
}

void setFunctional(bool key)
{
  if (key != isFunctional)
  {
    isFunctional = key;
    reportInfo("KinovaAPIService", "update", isFunctional, emergencyMode);
  }
}

void setAngleOffset(const std_msgs::Float32MultiArray::ConstPtr& offsets)
{
      angleOffsets[0] = offsets->data[0];
      angleOffsets[1] = offsets->data[1];
      angleOffsets[2] = offsets->data[2];
      angleOffsets[3] = offsets->data[3];
}

void setTorqueZero(const std_msgs::Int32::ConstPtr& msg)
{
      (*MySetTorqueZero)(msg->data);
}

bool isAPICorrupted(){

  return (MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) || (MyGetAngularPosition == NULL) || (MyGetDevices == NULL) ||
          (MySetActiveDevice == NULL) || (MyRunGravityZEstimationSequence == NULL) || (MySwitchTrajectoryTorque == NULL) || (MySetTorqueSafetyFactor == NULL) ||
          (MySendAngularTorqueCommand == NULL) || (MySendBasicTrajectory == NULL) || (MySetGravityVector == NULL) || (MySetGravityPayload == NULL) ||
          (MySetGravityOptimalZParam == NULL) || (MySetGravityType == NULL) || (MySetTorqueVibrationController == NULL) || (MyGetAngularForceGravityFree == NULL) ||
          (MySetTorqueControlType == NULL) || (MyGetAngularCurrentMotor == NULL) || (MyGetAngularForce == NULL) || (MyGetTemperature == NULL) ||
          (MyGetActuatorAcceleration == NULL) || (MyGetAngularVelocity == NULL);
}

void switchTorqueMode()
{
  MySwitchTrajectoryTorque(POSITION);
  MySetTorqueSafetyFactor(1);
  MySetTorqueVibrationController(0);
  MySwitchTrajectoryTorque(TORQUE);
  reportInfo("KinovaAPIService", "TorqueControlSwitch", isFunctional, emergencyMode);
}

void torqueCloseLoop(std::vector<float> torques)
{
    if (!emergencyMode)
    {
      reportError("KinovaCommandCallback", "CommandTrialInEmergencyMode", CommandTrialInEmergencyMode);
      return;
    }
    //If system is still active then:
    for (int i = 0; i < torques.size() ; i++)
          TorqueCommand[i] = torques.at(i);
    result = MySendAngularTorqueCommand(TorqueCommand);
    if (result != NO_ERROR_KINOVA)
    {
      setFunctional(false);
      reportError("KinovaCommandCallback", "Communication", CommandCommunicationFail);
      ROS_FATAL("KinovaAPIService:TorqueCommand:Device is not reachable.");
    }
    else
    {
      setFunctional(true);
    }
}

void commandCallback(const misa_files::KinovaCommandMessage& req)
{
    if (emergencyMode)
    {
      reportError("KinovaCommandCallback", "CommandTrialInEmergencyMode", CommandTrialInEmergencyMode);
      return;
    }
    string commandName = req.name;
    if (commandName.compare("angular") == 0)
    {
        string controlType = req.type;

        if (controlType.compare("position") == 0)
        {
              if(currentControlType != "angular_position")
              {
                pointToSend.Position.Type = ANGULAR_POSITION;
                reportInfo("KinovaAPIService", "PositionControlSwitch", isFunctional, emergencyMode);
                currentControlType = "angular_position";
                enableTorqueCloseLoop = false;
              }
              pointToSend.Position.Actuators.Actuator1 = (req.data.data[0] + angleOffsets[0]);
              pointToSend.Position.Actuators.Actuator2 = (req.data.data[1] + angleOffsets[1]);
              pointToSend.Position.Actuators.Actuator3 = (req.data.data[2] + angleOffsets[2]);
              pointToSend.Position.Actuators.Actuator4 = (req.data.data[3] + angleOffsets[3]);
              result = MySendBasicTrajectory(pointToSend);
              if (result != NO_ERROR_KINOVA)
              {
                setFunctional(false);
                reportError("KinovaCommandCallback", "CommandCommunicationFail", CommandCommunicationFail);
                ROS_FATAL("KinovaAPIService:AngularPosition:Device is not reachable.");
              }
              else
              {
                setFunctional(true);
              }

        }
        else if (controlType.compare("velocity") == 0)
        {
          if(currentControlType != "angular_velocity")
          {
              pointToSend.Position.Type = ANGULAR_VELOCITY;
              reportInfo("KinovaAPIService", "VelocityControlSwitch", isFunctional, emergencyMode);
              currentControlType = "angular_velocity";
              enableTorqueCloseLoop = false;
          }
              pointToSend.Position.Actuators.Actuator1 = (req.data.data[0]);
              pointToSend.Position.Actuators.Actuator2 = (req.data.data[1]);
              pointToSend.Position.Actuators.Actuator3 = (req.data.data[2]);
              pointToSend.Position.Actuators.Actuator4 = (req.data.data[3]);

              result = MySendBasicTrajectory(pointToSend);
              if (result != NO_ERROR_KINOVA)
              {
                  setFunctional(false);
                  reportError("KinovaCommandCallback", "CommandCommunicationFail", CommandCommunicationFail);
                  ROS_FATAL("KinovaAPIService:AngularVelocity:Device is not reachable.");
              }
              else
              {
                  setFunctional(true);
              }
        }
        else if (controlType.compare("torque") == 0)
        {
              if(currentControlType != "angular_torque")
              {
                  currentControlType = "angular_torque";
                  enableTorqueCloseLoop = true;
                  torqueArray = req.data.data;
                  switchTorqueMode();
                  //Set angular torque control
                  /*Not implemented yet.
                  * This will activate torque control mode
                  *   Torque Control mode
                  *     Loop that sends the same torque values that took in this block.
                  */
              }
              torqueCloseLoop(req.data.data);
        }
        else
        {
          ROS_WARN("KinovaAPIService::Wrong angular command type.");
          reportError("KinovaCommandCallback", "InvalidCommandType", InvalidCommandType);
        }

      }
      else
      {
          ROS_WARN("KinovaAPIService::Wrong command name.");
          reportError("KinovaCommandCallback", "InvalidCommandName", InvalidCommandName);
      }
}

bool sensorServiceHandler(misa_files::KinovaSensorService::Request &req, misa_files::KinovaSensorService::Response &res)
{
    if(req.sensor_name.compare("positions") == 0)
    {
          result = (*MyGetAngularPosition)(angleData);
          if(result == NO_ERROR_KINOVA)
          {
                setFunctional(true);
                res.response.header.stamp = ros::Time::now();
                if (initial){
                    initial = false;
                    angleOffsets[0] = angleData.Actuators.Actuator1;
                    angleOffsets[1] = angleData.Actuators.Actuator2;
                    angleOffsets[2] = angleData.Actuators.Actuator3;
                    angleOffsets[3] = angleData.Actuators.Actuator4;
                }
                res.response.name = "positions";
                res.response.data.data.clear();
                res.response.data.data.push_back( angleData.Actuators.Actuator1 - angleOffsets[0]);
                res.response.data.data.push_back( angleData.Actuators.Actuator2 - angleOffsets[1]);
                res.response.data.data.push_back( angleData.Actuators.Actuator3 - angleOffsets[2]);
                res.response.data.data.push_back( angleData.Actuators.Actuator4 - angleOffsets[3]);
                return true;
          }
          else
          {
                setFunctional(false);
                reportError("SensorServiceHandler", "Communication", SensorCommunicationFail);
                ROS_FATAL("KinovaAPIService : Device is not reachable.");
                return false;
          }
      }
      else if(req.sensor_name.compare("currents") == 0)
      {
                //Currents of the motors.
              result = (*MyGetAngularCurrentMotor)(currentData);
              if(result == NO_ERROR_KINOVA)
              {
                      setFunctional(true);
                      res.response.header.stamp = ros::Time::now();
                      res.response.name = "currents";
                      res.response.data.data.clear();
                      res.response.data.data.push_back( currentData.Actuators.Actuator1);
                      res.response.data.data.push_back( currentData.Actuators.Actuator2);
                      res.response.data.data.push_back( currentData.Actuators.Actuator3);
                      res.response.data.data.push_back( currentData.Actuators.Actuator4);
              }
              else
              {
                      setFunctional(false);
                      reportError("SensorServiceHandler", "Communication", SensorCommunicationFail);
                      ROS_FATAL("KinovaAPIService : Device is not reachable.");
                      return false;
              }
        }
        else if(req.sensor_name.compare("torques") == 0)
        {
                result = (*MyGetAngularForce)(torqueData);
                if(result == NO_ERROR_KINOVA)
                {
                      setFunctional(true);
                      res.response.header.stamp = ros::Time::now();
                      res.response.name = "torques";
                      res.response.data.data.clear();
                      res.response.data.data.push_back( torqueData.Actuators.Actuator1);
                      res.response.data.data.push_back( torqueData.Actuators.Actuator2);
                      res.response.data.data.push_back( torqueData.Actuators.Actuator3);
                      res.response.data.data.push_back( torqueData.Actuators.Actuator4);
                }
                else
                {
                      setFunctional(false);
                      reportError("SensorServiceHandler", "Communication", SensorCommunicationFail);
                      ROS_FATAL("KinovaAPIService : Device is not reachable.");
                      return false;
                }
        }
        else if(req.sensor_name.compare("temperatures") == 0)
        {
                result = (*MyGetTemperature)(temperatureData);
                if(result == NO_ERROR_KINOVA)
                {
                        setFunctional(true);
                        res.response.header.stamp = ros::Time::now();
                        res.response.name = "temperatures";
                        res.response.data.data.clear();
                        res.response.data.data.push_back( temperatureData.ActuatorTemp1);
                        res.response.data.data.push_back( temperatureData.ActuatorTemp2);
                        res.response.data.data.push_back( temperatureData.ActuatorTemp3);
                        res.response.data.data.push_back( temperatureData.ActuatorTemp4);
                }
                else
                {
                  setFunctional(false);
                  reportError("SensorServiceHandler", "Communication", SensorCommunicationFail);
                  ROS_FATAL("KinovaAPIService : Device is not reachable.");
                  return false;
                }
        }
        else if(req.sensor_name.compare("accelerations") == 0)
        {
                result = (*MyGetActuatorAcceleration)(accelerationData);
                if(result == NO_ERROR_KINOVA)
                {
                        setFunctional(true);
                        res.response.header.stamp = ros::Time::now();
                        res.response.name = "accelerations";
                        res.response.data.data.clear();

                        res.response.data.data.push_back(accelerationData.Actuator1_X);
                        res.response.data.data.push_back(accelerationData.Actuator1_Y);
                        res.response.data.data.push_back(accelerationData.Actuator1_Z);

                        res.response.data.data.push_back(accelerationData.Actuator2_X);
                        res.response.data.data.push_back(accelerationData.Actuator2_Y);
                        res.response.data.data.push_back(accelerationData.Actuator2_Z);

                        res.response.data.data.push_back(accelerationData.Actuator3_X);
                        res.response.data.data.push_back(accelerationData.Actuator3_Y);
                        res.response.data.data.push_back(accelerationData.Actuator3_Z);

                        res.response.data.data.push_back(accelerationData.Actuator4_X);
                        res.response.data.data.push_back(accelerationData.Actuator4_Y);
                        res.response.data.data.push_back(accelerationData.Actuator4_Z);
                        return true;
                }
                else
                {
                  setFunctional(false);
                  reportError("SensorServiceHandler", "Communication", SensorCommunicationFail);
                  ROS_FATAL("KinovaAPIService : Device is not reachable.");
                  return false;
                }
          }
          else if(req.sensor_name.compare("velocities") == 0)
          {
                result = (*MyGetAngularVelocity)(velocityData);
                if(result == NO_ERROR_KINOVA)
                {
                        setFunctional(true);
                        res.response.header.stamp = ros::Time::now();
                        res.response.name = "velocities";
                        res.response.data.data.clear();
                        res.response.data.data.push_back(velocityData.Actuators.Actuator1);
                        res.response.data.data.push_back(velocityData.Actuators.Actuator2);
                        res.response.data.data.push_back(velocityData.Actuators.Actuator3);
                        res.response.data.data.push_back(velocityData.Actuators.Actuator4);
                        return true;
                }
                else
                {
                  setFunctional(false);
                  reportError("SensorServiceHandler", "Communication", SensorCommunicationFail);
                  ROS_FATAL("KinovaAPIService : Device is not reachable.");
                  return false;
                }
        }
        else
        {
          reportError("SensorServiceHandler", "InvalidSensorName", InvalidSensorName);
          ROS_WARN("Wrong sensor request name.");
          return false;
        }

}

bool statusServiceHandler(misa_files::KinovaStatusService::Request &req, misa_files::KinovaStatusService::Response &res)
{
      res.response.APIActive = !isAPICorrupted();
      result = MyGetAngularCommand(DataCommand);
      //For now there is no limitation, except your imagination.
      res.response.limitation = false;
      if (result != NO_ERROR_KINOVA){

          setFunctional(true);
          res.response.communicationActive = true;
          int TrajectoryTorqueMode = 0;
          MyGetTrajectoryTorqueMode(TrajectoryTorqueMode);
          res.response.controlName = "angular";

          if (TrajectoryTorqueMode == 0)
          {
              if (currentControlType.compare("angular_position"))
                  res.response.controlType = "position";
              else
                  res.response.controlType = "velocity";

              res.response.trajectoryFIFO.data.clear();
              res.response.trajectoryFIFO.data.push_back(DataCommand.Actuators.Actuator1);
              res.response.trajectoryFIFO.data.push_back(DataCommand.Actuators.Actuator2);
              res.response.trajectoryFIFO.data.push_back(DataCommand.Actuators.Actuator3);
              res.response.trajectoryFIFO.data.push_back(DataCommand.Actuators.Actuator4);

          }
          else
          {
              res.response.controlType = "torque";
              MyGetAngularTorqueCommand(TorqueFIFO);
              res.response.trajectoryFIFO.data.push_back(TorqueFIFO[0]);
              res.response.trajectoryFIFO.data.push_back(TorqueFIFO[1]);
              res.response.trajectoryFIFO.data.push_back(TorqueFIFO[2]);
              res.response.trajectoryFIFO.data.push_back(TorqueFIFO[3]);
          }
      }
      else
      {
          res.response.communicationActive = false;
          setFunctional(false);
          reportError("KinovaAPIService", "Communication", StatusCommunicationFail);

      }
      return true;
}

void setFrequency(const std_msgs::Int32& msg)
{
    rosFrequency = msg.data;
}

void safeStop(){
    //Switch into the angular velocity mode.
    if(currentControlType != "angular_velocity")
    {
        pointToSend.Position.Type = ANGULAR_VELOCITY;
        currentControlType = "angular_velocity";
        enableTorqueCloseLoop = false;
    }
      pointToSend.Position.Actuators.Actuator1 = 0;
      pointToSend.Position.Actuators.Actuator2 = 0;
      pointToSend.Position.Actuators.Actuator3 = 0;
      pointToSend.Position.Actuators.Actuator4 = 0;

      result = MySendBasicTrajectory(pointToSend);
      if (result != NO_ERROR_KINOVA)
      {
          reportError("KinovaCommandCallback", "Communication", CommandCommunicationFail);
          setFunctional(false);
          ROS_FATAL("KinovaAPIService:AngularVelocity:Device is not reachable.");
      }
      else
      {
          setFunctional(true);
      }
}

void emergencyCheck(const std_msgs::Bool &msg)
{
  if (!msg.data)
  {
    emergencyMode = false;
    reportInfo("KinovaAPIService", "emergency_off", isFunctional, emergencyMode);
  }
  else
  {
    emergencyMode = true;
    safeStop();
    reportInfo("KinovaAPIService", "emergency_on", isFunctional, emergencyMode);
    ROS_WARN("KinovaAPIService:: Switched to EmergencyMode");
  }

}

void terminateSafely(int sig)
{
    safeStop();
    reportInfo("KinovaAPIService", "termination", isFunctional, emergencyMode);
    ROS_INFO("KinovaAPIService is being closed.");
    dlclose(commandLayer_handle);
    ros::shutdown();
}

int main(int argc, char **argv)
{
  //Ros Messages
  assert(!isAPICorrupted());

  //Control variable initialization
  pointToSend.InitStruct();
  pointToSend.Position.Type = ANGULAR_POSITION;
  currentControlType = "angular_position";
  accelerationData.InitStruct();
  emergencyMode = false;
  for (int i = 0; i < COMMAND_SIZE; i++)
  {
      TorqueCommand[i] = 0;
  }

  //Init ROS
  ros::init(argc, argv, "KinovaAPI");

  ros::NodeHandle n;

  initial = true;

  //Services
  ros::ServiceServer sensorService = n.advertiseService("KinovaSensorService", sensorServiceHandler);
  ros::ServiceServer statusService = n.advertiseService("KinovaStatusService", statusServiceHandler);

  //Subscribers
  ros::Subscriber commandService = n.subscribe("kinova_command", 100, commandCallback);
  ros::Subscriber setTorqueZeroSubsriber = n.subscribe("kinova_set_torque_zero", 10, setTorqueZero);
  ros::Subscriber changeRosFrequency = n.subscribe("kinova_set_frequency", 5, setFrequency);
  ros::Subscriber emergency = n.subscribe("kinova_emergency_stop", 5, emergencyCheck);

  //Publishers
  errorPub = n.advertise<misa_files::KinovaErrorMessage>("kinova_errors", 20);
  infoPub = n.advertise<misa_files::KinovaInfoMessage>("kinova_system_info",20);
  rosFrequency = 100;
  //ros::Rate loop_rate(rosFrequency);
  ros::Rate loopRate(rosFrequency);
  result = (*MyInitAPI)();
  if (result == NO_ERROR_KINOVA)
      isFunctional = true;
  else
      isFunctional = false;

  signal(SIGINT, terminateSafely);
  reportInfo("KinovaAPIService", "setup", isFunctional, emergencyMode);
  while( ros::ok() )
  {
    if(enableTorqueCloseLoop && !emergencyMode) torqueCloseLoop(torqueArray);
    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}
