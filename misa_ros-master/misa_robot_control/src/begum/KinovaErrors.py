from enum import Enum

class KinovaErrors(Enum):
    InvalidCommandName = 2
    InvalidSensorName = 2
    InvalidCommandType = 2

    CommandTrialInEmergencyMode = 3
    Termination = 4

    CommandSubscribeFail = 5
    SensorServiceResponseFail = 5

    CommandCommunicationFail = 6
    SensorCommunicationFail = 6
    EmergencyMode = 7
    APICorrupted = 10
    RosBroken = 20
