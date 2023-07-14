#ifndef __KINOVAERRORS__
#define __KINOVAERRORS__

enum
{
  InvalidCommandName = 2,
  InvalidSensorName = 2,
  InvalidCommandType = 2,

  CommandTrialInEmergencyMode = 3,
  Termination = 4,

  CommandCommunicationFail = 6,
  SensorCommunicationFail = 6,
  StatusCommunicationFail = 6,
  EmergencyMode = 7,

  APICorrupted = 10,
  RosBroken = 20

};

#endif
