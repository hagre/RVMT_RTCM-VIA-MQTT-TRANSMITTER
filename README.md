# RVMT-RTCM-VIA-MQTT-TRANSMITTER-
INFO: This Software is made for my private use. I like to share, but for further details read the LICENCE file! (GPL3)

RVMT (RTCM VIA MQTT TRANSMITTER) is using the MQTT protocol (as a secure and opensource alternative to NTRIP) to get RTK correction data for my rover GPS units.
As a target system i will use https://github.com/farmerbriantee/AgOpenGPS.

This is a ESP32 2 in 1 firmware file.
Basic selection of features via #define in the top of this code

#define SERVER_ROOF_NOTE
#1 - Transforming RTCM3 serial input into seperated MQTT Msg into differnet Topics. 
    - Send to Broker (Server) with or without secure connection (TLS) and/or Username and Password protected.
    - Topics are for example /NTRIP/Base/XYZ01/RTCM/1005/ or /NTRIP/Base/XYZ01/RTCM/1074/
    - Input can be switched of with a simple ON or OFF msg to e.g "/NTRIP/Base/XYZ01/Commands/"
    - 1005 msg is only transmitted all 20 seconds
    - Debug via USB can be enabled

#define CLIENT_ROVER_NOTE
#2 - Subscribing to MQTT Broker and wildcard Base station Topic /NTRIP/Base/XYZ01/RTCM/#
  - all received msg will arrive on the selected serial port and can be feed in to the f9p GPS (or somthing else)

All connections are currently done over wifi (first preperations are made to implement a Ethernet/LAN connection)

Nearly all parameters are adjusable in the first lines of code in the top #define section.
-Timings, IPs, Serialports, Buffersizes, Topics, ... adjust as required for your hardware (some hints or my last settings can be found behind the next // comment)

I am compiling witch VSCode and PlatformIO in the Arduino framwork.
Hardware in use:
-ESP32 noname board in the shape of an standard arduino UNO
-Ardusimple "simpleRTK2B v1.1" board with u-blox ANN-MB-00 Antenna for GNSS Dual Band

Big thanks to the arduino community for making ths all possible.
I have to anounce that an external libarie is used: knolleary/pubsubclient  (licensed under the MIT License). Check https://github.com/knolleary/pubsubclient

During research for this project, i found some projects working on ths promissing concept.

initialy found
https://github.com/GeoscienceAustralia/gnss-mqtt

and
http://www.ignss2018.unsw.edu.au/sites/ignss2018/files/u80/Slides/D2-S3-ThC-Wang.pdf

and comertialy
https://esprtk.com/

just found:
https://github.com/geerdkakes/mqtt-rtk

it would be nice if mqtt could be implemented to
https://github.com/nebkat/esp32-xbee 


ToDo:
-Find a good way to update of root_ca 
-Find a better and futureproof name for this new RTCM transmitting and hosting standard!
-complete the design of using the MQTT broker as an correction data server according to an NTRIP Caster

done by hagre 
06 2019 - 2020 
