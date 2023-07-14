
import rospy
from misa_files.msg import KinovaInfoMessage
from FileManager import FileManager

class InfoLogger:

    def __init__(self, owner_name):
        self._info_pub = rospy.Publisher("kinova_system_info", KinovaInfoMessage, queue_size = 5)
        self._info_msg = KinovaInfoMessage()
        self._master_name = "Logs"
        self._owner_name = owner_name
        self._file_manager = FileManager(master_name = "Logs", subfolder_name = owner_name)

    def report_info(self,source , reason = "update", is_functional = True, emergency_mode = False):
        self._info_msg.header.stamp = rospy.Time.now()
        self._info_msg.source = source
        self._info_msg.reason = reason
        self._info_msg.functional = is_functional
        self._info_msg.emergency = emergency_mode
        self.info_to_file(self._info_msg)
        self._info_pub.publish(self._info_msg)

    def info_to_file(self, info_msg):
        line = str(info_msg.header.stamp)
        line += " " + str(info_msg.source)
        line += " " + str(info_msg.reason)
        line += " " + str(info_msg.functional)
        line += " " + str(info_msg.emergency) + "\n"
        self._file_manager.write_line(line)

    def safe_exit(self):
        self._file_manager.close_file()
