from kinova_files.srv import *
from kinova_files.msg import *

import std_msgs

import rospy

msg = KinovaSensorMessage()

def handle_sensor_cb(req):
    global msg
    print ("Requested for " + req.sensor_name + " sensor.")
    print "Returning"
    msg.data.data = [1,2,3,4,5]
    msg.header = std_msgs.msg.Header()
    msg.header.stamp = rospy.Time.now()
    return KinovaSensorServiceResponse(msg)

def kinova_sensor_server():
    rospy.init_node('kinova_service')
    s = rospy.Service('KinovaSensorService', KinovaSensorService, handle_sensor_cb)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    kinova_sensor_server()
