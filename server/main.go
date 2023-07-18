package main

import (
	"fmt"

	"github.com/yigitoo/surgical-assistant-robot/server/lib"
)

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
