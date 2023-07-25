package lib

import (
	"net/http"

	"github.com/gin-gonic/gin"
	zmq "github.com/pebbe/zmq4"
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
	if err = requester.Connect("tcp://localhost:456"); err != nil {
		DirectLogError(err)
	}

	// Send the message to the ZeroMQ server
	request_payload := cmd.Name + ";" + cmd.Value

	if _, err = requester.Send(request_payload, 0); err != nil {
		DirectLogError(err)
	}

	// Wait for the response from the server (optional)
	reply, err := requester.Recv(0)
	if err != nil {
		DirectLogError(err)
		ctx.JSON(http.StatusInternalServerError, gin.H{
			"response": "NULL",
			"success":  false,
		})
	}

	ctx.JSON(http.StatusOK, gin.H{
		"response": reply,
		"success":  true,
	})
}
