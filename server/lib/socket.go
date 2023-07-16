package lib

import (
	"time"

	zmq "github.com/pebbe/zmq4"
)

func SendRobot(command_name string) any {
	InitializeLogger()

	responder, err := zmq.NewSocket(zmq.REP)
	LogError(err)
	defer responder.Close()
	responder.Bind("tcp://*:5555")

	for {
		msg, err := responder.Recv(0)
		LogError(err)
		InfoLogger.Println("Received: ", msg)

		time.Sleep(2 * time.Second)

	}
}
