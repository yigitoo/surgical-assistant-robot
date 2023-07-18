CC = g++

BIN_FOLDER = bin
BIN_NAME = robot_arm
fn = main.cc
FILE_NAME = $(fn)

ARGS = -Iinclude/ -ldl -lzmq -w

all: clean build run

build:
	mkdir $(BIN_FOLDER)
	$(CC) -o $(BIN_FOLDER)/$(BIN_NAME) src/$(FILE_NAME) $(ARGS) 
clean:
	rm -rf $(BIN_FOLDER)
run: 
	$(BIN_FOLDER)/$(BIN_NAME)

go:
	cd server
	go run cmd/main.go

.PHONY: all build run clean go