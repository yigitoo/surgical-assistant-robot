#!/bin/sh

cp -r ./lib/* /usr/lib/
cp -r ./arduino_libraries/ ~/Arduino/libraries/
# Look at the package name to suitable with your Operating System
# Link: https://zeromq.org/download/
apt-get update
apt-get upgrade
apt-get install libzmq3-dev
pip3 install -r requirements.txt
cd server
go mod tidy
cd ..
make build
