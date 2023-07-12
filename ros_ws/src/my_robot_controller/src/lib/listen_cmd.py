from .logger import LogLevel, LogLevelHandlerError

import rospy
import std_msgs.msg

class CmdListener():
    def __init__(self, node_name = "listen_cmd", msg_type = std_msgs.msg.String):
        self.node_name = node_name
        self.msg_type = msg_type
        rospy.init_node(self.node_name, anonymous=True)
        self.current_value = None

    def send_cmd(self, cmd):
        rospy.loginfo("Sending command: %s" % cmd)
        self.pub.publish(std_msgs.msg.String(cmd))
        rospy.loginfo("Request sended!")

    def listener_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        self.current_value = data.data

    def listener(self):
        rospy.Subscriber('command', std_msgs.msg.String, self.listener_callback)
        if type(std_msgs.msg.UInt16(self.current_value)) == std_msgs.msg.UInt16:
            self.log_info("true")
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def log_msg(self, level: LogLevel, msg: str):
        if level == LogLevel.INFO:
            rospy.loginfo(msg)
        elif level == LogLevel.WARN or level == LogLevel.WARNING:
            rospy.logwarn(msg)
        elif level == LogLevel.ERROR:
            rospy.logerr(msg)
        else:
            raise LogLevelHandlerError("Cannot create a log with log level %s" % level)

    def log_info(self, msg: str):
        self.log_msg(LogLevel.INFO, msg)

    def log_warn(self, msg: str):
        self.log_msg(LogLevel.WARN, msg)

    def log_error(self, msg: str):
        self.log_msg(LogLevel.ERROR, msg)

    def spin(self):
        rospy.spin()

    def is_shutdown(self):
        return rospy.is_shutdown()
