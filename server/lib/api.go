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

	router := gin.Default()
	router.SetTrustedProxies([]string{"0.0.0.0"})
	router.Static("/public", "./public")

	router.LoadHTMLGlob("templates/*.html")

	InfoLogger.Println("Routes are setting into application.")

	router.GET("/", func(ctx *gin.Context) {
		ctx.HTML(http.StatusOK, "index.html", gin.H{
			"content": "This is an index page...",
		})
		InfoLogger.Println("GET / HTTP/1. 200")
	})

	router.GET("/commander", func(ctx *gin.Context) {
		ctx.HTML(http.StatusOK, "commander.html", gin.H{
			"content": "This is an index page...",
		})
		InfoLogger.Println("GET / HTTP/1. 200")
	})

	router.GET("/api", func(ctx *gin.Context) {
		ctx.HTML(http.StatusOK, "api.html", gin.H{
			"content": "This is an api page...",
		})
		InfoLogger.Println("GET /api HTTP/1. 200")
	})

	InfoLogger.Println("Route / (GET HTTP/1.1) has been setted!")

	router.POST("/", SendCommand)
	InfoLogger.Println("Route / (POST [COMMAND_HANDLER] HTTP/1.1) has been setted!")

	return router
}
