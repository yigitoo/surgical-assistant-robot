import zmq
import json
import time

def apply(params):
    open('param.json','w').write(json.dumps(params))

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://127.0.0.1:5555")

while True:
    message = socket.recv()
    print(message)
    print("Received request: %s" % message)
    socket.send_string(str(jd))
    time.sleep(1)
