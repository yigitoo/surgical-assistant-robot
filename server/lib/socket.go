package lib

import (
	"encoding/json"
	"fmt"
	"net/http"

	"github.com/gin-gonic/gin"
	zmq "github.com/pebbe/zmq4"
	"github.com/tidwall/gjson"
)

// Message represents a simple message struct
type Command struct {
	Name  string `json:"cmd_name"`
	Value string `json:"cmd_val"`
}

type AllInfos struct {
	Angles       []float64     `json:"angles"`
	Cartesian    CartesianInfo `json:"cartesian"`
	Temperatures []float64     `json:"temperatures"`
	Currents     []float64     `json:"currents"`
	Torques      []float64     `json:"torques"`
}

type Motor struct {
	ID          int           `json:"id"`
	Angle       float64       `json:"angle"`
	Cartesian   CartesianInfo `json:"cartesian"`
	Temperature float64       `json:"temperature"`
	Current     float64       `json:"current"`
	Torque      float64       `json:"torque"`
}

type CartesianInfo struct {
	X      float64 `json:"x"`
	Y      float64 `json:"y"`
	Z      float64 `json:"z"`
	ThetaX float64 `json:"thetaX"`
	ThetaY float64 `json:"thetaY"`
	ThetaZ float64 `json:"thetaZ"`
}

type Response struct {
	Reply string `json:"reply"`
}

/*
Response:

	{
		"reply": "{raw_data_json #değişcek}"
	}
*/
func GetMotorById(motor_id string) (Motor, error) {
	var motor Motor
	var err error
	var socket *zmq.Socket
	socket, err = zmq.NewSocket(zmq.REQ)
	if err != nil {
		return motor, err
	}
	defer socket.Close()
	socket.Connect("tcp://localhost:5555")

	cmd := Command{
		Name:  "get_motor_by_id",
		Value: motor_id,
	}

	request_payload := cmd.Name + ";" + cmd.Value

	if _, err = socket.Send(request_payload, 0); err != nil {
		DirectLogError(err)
	}

	reply, err := socket.Recv(0)
	LogError(err)

	err = json.Unmarshal([]byte(reply), &motor)
	LogError(err)

	return motor, err
}

func DenemeMotorID(motor_id string) (string, error) {

	var err error
	var socket *zmq.Socket
	socket, err = zmq.NewSocket(zmq.REQ)
	if err != nil {
		return "", err
	}
	defer socket.Close()
	socket.Connect("tcp://localhost:5555")

	cmd := Command{
		Name:  "get_motor_by_id",
		Value: motor_id,
	}

	request_payload := cmd.Name + ";" + cmd.Value

	if _, err = socket.Send(request_payload, 0); err != nil {
		DirectLogError(err)
	}

	reply, err := socket.Recv(0)
	LogError(err)
	fmt.Println(reply)
	replyMessage := gjson.Get(reply, "reply")
	return replyMessage.String(), nil
}

// SendCommand handles sending the message via ZeroMQ
func SendCommand(ctx *gin.Context) {
	// Get the message from the request body
	var cmd Command
	if err := ctx.ShouldBindJSON(&cmd); err != nil {
		ctx.JSON(400, gin.H{
			"error": "Invalid request for json, use cmd_name for command name post request.",
		})
		return
	}

	// Create ZeroMQ socket for REQ-REP pattern
	requester, err := zmq.NewSocket(zmq.REQ)
	LogError(err)
	defer requester.Close()

	// Connect to the ZeroMQ server
	if err = requester.Connect("tcp://localhost:5555"); err != nil {
		DirectLogError(err)
	}

	// Send the message to the ZeroMQ server

	request_payload := cmd.Name + ";" + cmd.Value

	if _, err = requester.Send(request_payload, 0); err != nil {
		DirectLogError(err)
	}

	// Wait for the response from the server (optional)
	reply, err := requester.Recv(0)
	LogError(err)

	ctx.JSON(http.StatusOK, gin.H{
		"response": reply,
	})
}
