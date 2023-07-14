/*
void angularTorqueCommandCallback(const std_msgs::Float32MultiArray::ConstPtr& torqueCommandMessage)
{
    cout << "Angular Torque Command Callback." << endl;
    if ( generalControlType != TORQUE)
    {
      //Change it into Torque
          MySwitchTrajectoryTorque(TORQUE);
          generalControlType = TORQUE;
    }
}

void angularControlCommandCallback(const std_msgs::Float32MultiArray::ConstPtr& angularBasicTrajectory)
{
			pointToSend.Position.Actuators.Actuator1 = (angularBasicTrajectory->data[0] + angleOffsets[0]);
      pointToSend.Position.Actuators.Actuator2 = (angularBasicTrajectory->data[1] + angleOffsets[1]);
      pointToSend.Position.Actuators.Actuator3 = (angularBasicTrajectory->data[2] + angleOffsets[2]);
      pointToSend.Position.Actuators.Actuator4 = (angularBasicTrajectory->data[3] + angleOffsets[3]);
			MySendBasicTrajectory(pointToSend);
}
*/

/*
void angControlTypeCallback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data.compare("angular_position") == 0)
  {
      pointToSend.Position.Type = ANGULAR_POSITION;
      cout << "Angular position is set." << endl;
  }
  if(msg->data.compare("angular_velocity") == 0)
  {
      pointToSend.Position.Type = ANGULAR_VELOCITY;
      cout << "Angular velocity is set." << endl;
  }
}

void setTorqueVibrationCallback(const std_msgs::Float32::ConstPtr& torqueVibrationMessage)
{
    if ( torqueControlType != DIRECTTORQUE )
    {
          MySetTorqueControlType(DIRECTTORQUE);
          torqueControlType = DIRECTTORQUE;
    }
    MySetTorqueVibrationController(torqueVibrationMessage->data);

}

void setTorqueSafetyFactorCallback(const std_msgs::Float32::ConstPtr& torqueSafetyFactorMessage)
{
    cout << "Set Safety Factor Callback" << endl;
    if ( torqueControlType != DIRECTTORQUE )
    {
          MySetTorqueControlType(DIRECTTORQUE);
          torqueControlType = DIRECTTORQUE;
    }
    MySetTorqueSafetyFactor(torqueSafetyFactorMessage->data);

}
*/
//Init Subscribers
/*
ros::Subscriber angularTorqueCommandSubsriber = n.subscribe("km_angular_torque_command", 1000, angularTorqueCommandCallback);
ros::Subscriber setTorqueSafetyFactorSubscriber = n.subscribe("km_torque_safety_factor", 1000, setTorqueSafetyFactorCallback);
ros::Subscriber setTorqueVibrationSubscriber = n.subscribe("km_torque_vibration_factor", 1000, setTorqueVibrationCallback);
ros::Subscriber angControlSubscriber = n.subscribe("km_angular_control", 10, angularControlCommandCallback);
ros::Subscriber angControlTypeSubscriber = n.subscribe("km_angular_control_type", 10, angControlTypeCallback);
ros::Subscriber setOffsetSubsriber = n.subscribe("km_set_angle_offsets", 10, setAngleOffset);

*/
