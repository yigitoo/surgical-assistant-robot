package lib

type AllInfos struct {
	Actuators Actuators
}

type Actuators struct {
	AngularInfo AngularInfo
}

type Actuator struct {
}

type AngularInfo struct {
	// It is using degrees if it will be volocity it will use degrees per second.
	Actuator1 float64
	Actuator2 float64
	Actuator3 float64
	Actuator4 float64
	Actuator5 float64
	Actuator6 float64
	Actuator7 float64
}

type AngularInfoSpesificMotor struct {
	ActuatorID uint
	Angle      float64
}
