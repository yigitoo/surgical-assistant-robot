CC = g++

BIN_FOLDER = bin
BIN_NAME = robot_arm
fn = main
FILE_NAME = $(fn)

ARGS = -Iinclude/ -ldl -lzmq -Wall 

build:
	mkdir $(BIN_FOLDER)
	$(CC) -o $(BIN_FOLDER)/$(BIN_NAME) src/$(FILE_NAME).cc $(ARGS) 
clean:
	rm -rf $(BIN_FOLDER)
run: 
	$(BIN_FOLDER)/$(BIN_NAME)