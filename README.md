# surgical-assistant-robot
A surgical assistant robot's source code developed with Özyeğin University via Summer
intern (with Asst. Prof. Özkan Bebek).
<br>
<br>

# Requirements:

KinovaAPI: version 6.1.0
<br>

<b>libzmq-3:</b> ZeroMQ library for communication with backend server and C++ hardware receiver server.<a href="https://zeromq.org/download/"> You can download via this link.</a>

<br>

<b>Windows.h:</b> If you are using Windows (Install via Visual Studio)

<br>

<b>dlfcn.h:</b> If you are using Linux (i think it comes with gcc / like a std lib in C/C++)

<br>

<b>C++ compiler:</b> i preferred g++ compiler

<br>

<b>Golang:</b> 1.20 version of golang.<br>
Note: 1.20 is the default version for Golang API<br>
It's not tested on other versions.<br>
The stabilistic version is this.<br>
<br>
<br>

---

<br>

# The project structure is:
<pre>
---

src
|
\----> main.cc is used for to communcate between commander and hardware. (Kinova KA-75+ & KA-58 Actuators)
                                    (We used this actuators for moving surgical-assistant-robot)

---

include
|
\----> Third party headers are included in the side of C++ code.
\----> Kinova KA-75+ & KA-58 Actuators header files. (That's not enoug for usage you need .so dynamic libraries to use them.)
\----> RapidJSON: It is a third party json handling library developed by Tencent Coorparation. (The PUBG one)

---

bin
|
\----> Build directory for our source files.

---

server | GoLang Projexy Server for communication with backend server and C++ hardware sender/receiver server.
|
\----> main.go | Main file for application server
       lib
       |
       \----> api.go | The head of the project structure and the api handlers for the application server.
       \----> logger.go | The logger for the application server.
       \----> config.go | The config for the application server.

---

current directory
|
\----> main.py | It is used for commander app (Coded in Flask).
\----> setup.sh | Main setup bash script for install dependencies of this project.

---

templates | Folder for holding HTML5 templates for Flask (Python3 micro backend framework)
|
\----> api.html | A little bit of documentation of GoLang server usage.
\----> commander.html | Remote Control Unit User Interface for Commander.
\----> index.html | That pages used for choosing api.html or commander.html 
       (Kinda used for navigating between pages.)

---

public | Folder for holding CSS3 and JavaScript files for Flask Server. (Commander Desktop App Server)
|
\----> index.css | Static CSS file
\----> index.js | Static JavaScript file

---

arduino_libraries | It's used for servo engines to add that your Arduino IDE
use the bash script named `setup.sh`

---

...
\----> Other files are is used for experimantal settings.
</pre>
<br>

# Informations about project.

LICENSE: MIT License <br>
Maintainer: Asst. Prof. Özkan Bebek <br>
Maintainer's Asst's: Ahmet Burhan Kara, Süleyman Can Çevik.
Authors: Yiğit GÜMÜŞ, Salih Burak AYDOĞDU, Ozan GÖRGÜ, Bengisu PAPAKÇI
