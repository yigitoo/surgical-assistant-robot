#!/usr/bin/env python3
from send_cmd import CmdSender
from logger import LogLevel

if __name__ == '__main__':
    cmd_sender = CmdSender("cmd_sender")
    cmd_sender.set_rate(50)

    cmd_sender.send_cmd("Hello World!")
    cmd_sender.log_msg(LogLevel.INFO ,"\"Hello world\" msg sended to rosmaster!")
    cmd_sender.log_warn("I use that for debugging my logger.")
    while not cmd_sender.is_shutdown():
        cmd_sender.send_cmd("ADIOS!")
        cmd_sender.sleep()
