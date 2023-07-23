# surgical-assistant-robot
A surgical assistant robot's source code developed with Özyeğin University via Summer
intern (with Asst. Prof. Özkan Bebek).
<br>
<br>

# Requirements:
KinovaAPI: version 6.1.0
<br>
libzmq-3: ZeroMQ library for communication with backend server and C++ hardware receiver server.<a href="https://zeromq.org/download/"> You can download via this link.</a>
<br>
Windows.h: If you are using Windows (Install via Visual Studio)
<br>
dlfcn.h: If you are using Linux (i think it comes with gcc / like a std lib in C/C++)
<br>
C++ compiler: i preferred g++ compiler
<br>
Golang: 1.20 version of golang.
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
\--> main.py is used for commander app (Coded in Flask).
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
...
\----> Other files are is used for experimantal settings.
</pre>
<br>
---

# Informations about project.

LICENSE: MIT License <br>
Maintainer: Asst. Prof. Özkan Bebek <br>
Maintainer's Asst's: Ahmet Burhan Kara, Süleyman Can Çevik.
Authors: Yiğit GÜMÜŞ, Salih Burak AYDOĞDU, Ozan GÖRGÜ, Bengisu PAPAKÇI
