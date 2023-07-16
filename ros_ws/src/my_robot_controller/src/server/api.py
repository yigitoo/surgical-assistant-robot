import json
import zmq
import time



jd = json.load(open("param.json","r"))

print(jd)

""""
{
    "general": {
        "option":0
    },
    "Motors":
    [{
        "id":"0x0000000",
        "angle":0
    },
    {
        "id":"0x0000000",
        "angle":0
    }
]}
"""

def apply(params):
    open('param.json','w').write(json.dumps(params))

jd["general"]["option"] = 1

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://127.0.0.1:5555")

while True:
    message = socket.recv()
    print(message)
    print("Received request: %s" % message)
    socket.send_string(str(jd))
    time.sleep(1)
