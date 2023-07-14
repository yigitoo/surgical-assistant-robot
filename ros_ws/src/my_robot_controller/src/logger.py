from enum import Enum

class LogLevel(Enum):
    INFO = 1
    WARNING = 2
    WARN = 2
    ERROR = 3

class LogLevelHandlerError:
    loglvl = LogLevel.INFO
    def __init__(self, loglvl, err_msg = "Unknown error, log level = %s" % loglvl):
        self.err_msg = err_msg
        self.loglvl = loglvl
