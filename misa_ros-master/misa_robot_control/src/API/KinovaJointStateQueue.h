#ifndef __KINOVAJOINTSTATEQUEUE__
#define __KINOVAJOINTSTATEQUEUE__

#include "misa_files/KinovaJointStateMessage.h"

#include <queue>

class KinovaJointStateQueue
{
  private:

    //Private variables
    int queueSize;

    float summedPosition[4], summedVelocity[4], summedAcceleration[4], summedTorque[4];
    void clearTempMessage();
    void zerosTempMessage();
    misa_files::KinovaJointStateMessage tempMessage;
    misa_files::KinovaJointStateMessage messengerMessage;

    std::queue<misa_files::KinovaJointStateMessage> messageQueue;

  public:
    //Constructor and destructor
    KinovaJointStateQueue(int queueSize);
    ~KinovaJointStateQueue();

    //Public methods
    misa_files::KinovaJointStateMessage getFilteredMessage();
    void push(misa_files::KinovaJointStateMessage newMessage);
    void setQueueSize(int queueSize);
    int getQueueSize();
};


#endif
