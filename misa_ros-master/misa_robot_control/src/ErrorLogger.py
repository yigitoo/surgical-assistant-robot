
import rospy
from misa_files.msg import KinovaErrorMessage
from FileManager import FileManager

class ErrorLogger:

    def __init__(self, owner_name):
        self._err_pub = rospy.Publisher("kinova_errors", KinovaErrorMessage, queue_size = 10)
        self._err_msg = KinovaErrorMessage()
        self._master_name = "Errors"
        self._owner_name = owner_name
        self._file_manager = FileManager(master_name = "Errors", subfolder_name = owner_name)

    def err_to_file(self, err_msg):
        line = str(err_msg.header.stamp)
        line += " " + str(err_msg.source)
        line += " " + str(err_msg.error)
        line += " " + str(err_msg.priority) + "\n"
        self._file_manager.write_line(line)

    def report_error(self, source , error , priority):
        self._err_msg.header.stamp = rospy.Time.now()
        self._err_msg.source = source
        self._err_msg.error = error
        self._err_msg.priority = priority
        self._err_pub.publish(self._err_msg)
        self.err_to_file(self._err_msg)

    def safe_exit(self):
        self._file_manager.close()
