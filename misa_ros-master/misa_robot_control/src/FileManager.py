#!/usr/bin/env python

import rospy
import os
import datetime

class FileManager:
    def __init__(self, master_name, subfolder_name, auto_gen = True):

        self._subfolder_name = subfolder_name
        self._folder_name = self.generate_name_by_date()
        self._file_name = self.generate_name_by_time()

        self._master_wd = os.getcwd() + "/" + master_name
        self._main_wd = self._master_wd + "/" + self._folder_name
        self._sub_wd = self._main_wd + "/" + subfolder_name
        self._file_path = self._sub_wd + "/" + self._file_name

        if not os.path.exists(self._master_wd):
            os.mkdir(self._master_wd)

        if not os.path.exists(self._main_wd):
            os.mkdir(self._main_wd)

        if not os.path.exists(self._sub_wd):
            os.mkdir(self._sub_wd)

        if os.path.exists(self._file_path):
            self._file_id = open(self._file_path, "a")
        else:
            self._file_id = open(self._file_path, "a+")

        self._file_id.write("Session started at %d:%d:%d \n" %(datetime.datetime.now().hour, datetime.datetime.now().minute, datetime.datetime.now().second))
        rospy.on_shutdown(self.close_file)

    def write_line(self, line):
        self._file_id.write(line)

    def generate_name_by_date(self):
        #Order is month, day, year
        date = datetime.date.today()
        name = str(date.month) + "_" + str(date.day) + "_" + str(date.year)
        return name

    def generate_name_by_time(self):
        time = datetime.datetime.now()
        return str(time.hour) + "_" + str(time.minute) + "_" + str(time.second)

    def create_file(self, file_name):
        os.mkfile(file_name)

    def create_folder(self, folder_name):
        os.mkdir(os.getcwd() + "/" + folder_name)

    def close_file(self):
        self._file_id.write("Session ended at %d:%d:%d \n" %(datetime.datetime.now().hour, datetime.datetime.now().minute, datetime.datetime.now().second))
        self._file_id.close()
