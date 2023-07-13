CC = g++

BIN_FOLDER = bin
BIN_NAME = robot_arm

ARGS = -Iinclude/ -ldl -Wall 

build:
	mkdir $(BIN_FOLDER)
	$(CC) -o $(BIN_FOLDER)/$(BIN_NAME) src/main.cc $(ARGS) 
clean:
	rm -rf $(BIN_FOLDER)
run: 
	$(BIN_FOLDER)/$(BIN_NAME)