package lib

import (
	"fmt"
	"net/http"
	"strconv"

	"github.com/gin-gonic/gin"
)

var config Config = NewConfig()
var iPORT int16 = config.GetPort()
var PORT = fmt.Sprintf(":%s", strconv.Itoa(int(iPORT)))

func SetupApi() *gin.Engine {
	InitializeLogger()
	InfoLogger.Println("=== PROGRAM STARTED ===")

	r := gin.Default()
	r.SetTrustedProxies([]string{"0.0.0.0"})
	r.Static("/public/", "./public")
	r.LoadHTMLGlob("./templates/*.html")

	InfoLogger.Println("Routes are setting into application.")

	r.GET("/", func(ctx *gin.Context) {
		ctx.HTML(http.StatusOK, "index.html", gin.H{
			"content": "This is an index page...",
		})
		InfoLogger.Println("GET / HTTP/1. 200")
	})

	InfoLogger.Println("Route / (GET HTTP/1.1) has been setted!")

	r.POST("/", SendCommand)

	InfoLogger.Println("Route / (POST [COMMAND_HANDLER] HTTP/1.1) has been setted!")

	r.GET("/id/:motorid", func(ctx *gin.Context) {
		motor_id := ctx.Params.ByName("motorid")
		motor_info, err := DenemeMotorID(motor_id)
		if err != nil {
			LogError(err)
			ctx.JSON(http.StatusInternalServerError, gin.H{
				"motor_id":   motor_id,
				"motor_info": "",
				"error":      err.Error(),
				"success":    false,
				"status":     http.StatusInternalServerError,
			})

			return
		}

		ctx.JSON(http.StatusOK, gin.H{
			"motor_id":   motor_id,
			"motor_info": motor_info,
			"successful": true,
			"status":     http.StatusOK,
		})
	})

	InfoLogger.Println("Route /id/:motorid (GET [COMMAND_HANDLER] HTTP/1.1) has been setted!")

	return r
}
