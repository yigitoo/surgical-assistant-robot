CC = g++

BIN_FOLDER = bin
BIN_NAME = robot_arm
fn = robot_arm
FILE_NAME = $(fn)

ARGS = -Iinclude/ -ldl -lzmq -w

all: clean build run

build:
	$(CC) -o $(BIN_FOLDER)/$(BIN_NAME) src/$(FILE_NAME).cc $(ARGS) 
clean:
	rm -rf $(BIN_FOLDER)/$(BIN_NAME)
run: 
	$(BIN_FOLDER)/$(BIN_NAME)

python:
	cd src
	python3 $(fn).py
go:
	cd server
	go run cmd/main.go

.PHONY: all build run clean go