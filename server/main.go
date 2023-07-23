package main

import (
	"fmt"

	"github.com/yigitoo/surgical-assistant-robot/server/lib"
)

/*
lib folder is a package that contains source code of all api utilities

"socket.go" handle the communication between the c++ client using
ZeroMQ message broker.

"logger.go" handle the logging of messages and reqests/responses to a file named
`server.log`

"config.go" is a ordinary config file for project codebase.

"main.go" is a binary package that to be used as executable and compiling package.
*/

func main() {
	api := lib.SetupApi()

	fmt.Printf(`
	
-------------------------------------
Link: http://localhost%s/
Author: yigitoo <Yiğit GÜMÜŞ | gumusyigit101@gmail.com>
-------------------------------------


`, lib.PORT)

	api.Run(lib.PORT)
}
