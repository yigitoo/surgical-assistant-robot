package lib

import (
	"context"
	"fmt"
	"log"
	"net/http"
	"strconv"

	"github.com/gin-gonic/gin"
)

var config Config = NewConfig()
var iPORT int16 = config.GetPort()
var PORT = fmt.Sprintf(":%s", strconv.Itoa(int(iPORT)))

func SetupApi() *gin.Engine {
	context := context.TODO()

	InitializeLogger()
	config.SetApiKeys()

	InfoLogger.Println("===PROGRAM_STARTED===")

	r := gin.Default()
	r.SetTrustedProxies([]string{"0.0.0.0"})
	r.Static("/public/", "./public")
	r.LoadHTMLGlob("./templates/*.html")

	InfoLogger.Println("Routes are setting into application.")

	r.GET("/", func(ctx *gin.Context) {
		ctx.HTML(http.StatusOK, "index.html", gin.H{
			"content": "This is an index page...",
		})
	})

	r.GET("/dumpInfos", func(ctx *gin.Context) {
		success := true
		all_infos, err := FetchAllInfos(context)

		if err != nil {
			log.Fatal(err)
			success = false
		}

		if success {
			ctx.JSON(http.StatusOK, gin.H{
				"data":       all_infos,
				"successful": true,
				"status":     http.StatusOK,
			})
			return
		}

		ctx.JSON(http.StatusInternalServerError, gin.H{
			"succesful": false,
			"status":    http.StatusInternalServerError,
		})
	})

	r.GET("/id/:motorid", func(ctx *gin.Context) {
		motor_id := ctx.Params.ByName("motorid")
		motor, err := GetMotorById(motor_id)
		LogError(err)

		ctx.JSON(http.StatusOK, gin.H{
			"motor_id":   motor_id,
			"motor_info": motor,
			"successful": true,
			"status":     http.StatusOK,
		})
	})

	return r
}
