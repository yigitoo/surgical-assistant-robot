

#include "KinovaJointStateQueue.h"

  KinovaJointStateQueue::KinovaJointStateQueue(int queueSize)
  {
      this->setQueueSize(queueSize);
      this->zerosTempMessage();

  }
  KinovaJointStateQueue::~KinovaJointStateQueue()
  {
      delete &messageQueue;
      delete &tempMessage;
      delete &messengerMessage;
  }

  misa_files::KinovaJointStateMessage KinovaJointStateQueue::getFilteredMessage()
  {
      //return this->queue.pop();
      this->clearTempMessage();

      int size = this->messageQueue.size();
      for (int i = 0; i < 4; i++)
      {
          this->messengerMessage.positions.push_back(summedPosition[i] / size);
          this->messengerMessage.velocities.push_back(summedVelocity[i] / size);
          this->messengerMessage.accelerations.push_back(summedAcceleration[i] / size);
          this->messengerMessage.torques.push_back(summedTorque[i] / size);
      }
      this->messengerMessage.header = this->messageQueue.back().header;
      return this->messengerMessage;
  }
  void KinovaJointStateQueue::clearTempMessage()
  {
    this->messengerMessage.positions.clear();
    this->messengerMessage.velocities.clear();
    this->messengerMessage.accelerations.clear();
    this->messengerMessage.torques.clear();

  }

  void KinovaJointStateQueue::zerosTempMessage()
  {
      this->clearTempMessage();
      for(int i = 0; i < 4 ; i++)
      {
          this->tempMessage.positions.push_back(0);
          this->tempMessage.velocities.push_back(0);
          this->tempMessage.accelerations.push_back(0);
          this->tempMessage.torques.push_back(0);
      }
  }

  void KinovaJointStateQueue::push(misa_files::KinovaJointStateMessage newMessage)
  {
        messageQueue.push(newMessage);

        if(this->queueSize < messageQueue.size())
        {
            this->tempMessage = messageQueue.front();
            messageQueue.pop();
        }
        else
        {
            this->zerosTempMessage();
        }

        for (int j = 0; j < 4; j++)
        {
            this->summedPosition[j] -= this->tempMessage.positions.at(j) - newMessage.positions.at(j);
            this->summedVelocity[j] -= this->tempMessage.velocities.at(j) - newMessage.velocities.at(j);
            this->summedAcceleration[j] -= this->tempMessage.accelerations.at(j) - newMessage.accelerations.at(j);
            this->summedTorque[j] -= this->tempMessage.torques.at(j) - newMessage.torques.at(j);

        }
  }
  void KinovaJointStateQueue::setQueueSize(int queueSize)
  {
      this->queueSize = queueSize;
  }
  int KinovaJointStateQueue::getQueueSize()
  {
      return this->queueSize;
  }
