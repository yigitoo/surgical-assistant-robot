from logger import LogLevel, LogLevelHandlerError
from std_msgs import msg
import rospy

class CmdSender():
    def __init__(self, node_name = "send_cmd", msg_type = msg.String):
        self.node_name = node_name
        self.msg_type = msg_type
        rospy.init_node(self.node_name, anonymous=True)
        self.pub = rospy.Publisher("/cmd_publisher", self.msg_type, queue_size=10)

    def send_cmd(self, cmd):
        rospy.loginfo("Sending command: %s" % cmd)
        self.pub.publish(self.msg_type(cmd))
        rospy.loginfo("Request sended!")

    def sleep(self):
        self.rate.sleep()

    def set_rate(self, rate: float):
        self.rate_val = rate
        self.rate = rospy.Rate(self.rate_val)

    def get_rate(self):
        return self.rate_val

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
