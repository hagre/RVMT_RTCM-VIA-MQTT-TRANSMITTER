
# DO NOT USE THIS VERSION - DEBUG IN PROGRESS

# RVMT (RTCM-VIA-MQTT-TRANSMITTER)
INFO: This Software is made for my private use. I like to share, but for further details read the LICENCE file! (GNU GPL V3)

Check:  

RVMR (RTCM-VIA-MQTT-RECEIVER) https://github.com/hagre/RVMR_RTCM-VIA-MQTT-RECEIVER

RVMP (RTCM-VIA-MQTT-PROTOCOL) https://github.com/hagre/RVMP_RTCM-VIA-MQTT-PROTOCOL 

?(RVMC (RTCM-VIA-MQTT-CASTER) is on https://github.com/hagre/RVMC_RTCM-VIA-MQTT-CASTER)

RVMT (RTCM VIA MQTT TRANSMITTER) is using the MQTT protocol (as a secure and opensource alternative to NTRIP) to get RTK correction data for my rover GPS units.
As a target system i will use https://github.com/farmerbriantee/AgOpenGPS.

This is a ESP32 firmware
Basic selection of features via #define in the top of this code

This TRANSMITTER is working like a "NTRIP-Base" and transmitting the RTCM data via MQTT to a broker.
* Transforming RTCM3 serial input into seperated MQTT Msg into differnet Topics. 
* Send to Broker (Server) with or without secure connection (TLS) and/or Username and Password protected.
* Topics are for example /RTK/Base/XYZ01/RTCM/1005/ or /RTK/Base/XYZ01/RTCM/1074/
* Input can be switched of with a simple ON or OFF msg to e.g "/RTK/Base/XYZ01/Commands/"
* Debug via USB can be enabled


# General
All connections are currently done over wifi (first preperations are made to implement a Ethernet/LAN connection)

Nearly all parameters are adjusable in the first lines of code in the top #define section.
-Timings, IPs, Serialports, Buffersizes, Topics, ... adjust as required for your hardware (some hints or my last settings can be found behind the next // comment)

I am compiling witch VSCode and PlatformIO in the Arduino framwork.
Hardware in use:
-ESP32 noname board in the shape of an standard arduino UNO
-Ardusimple "simpleRTK2B v1.1" board with u-blox ANN-MB-00 Antenna for GNSS Dual Band

Big thanks to the arduino community for making ths all possible.

I have to anounce that an external libarie is used: 
knolleary/pubsubclient  (licensed under the MIT License). 
Check https://github.com/knolleary/pubsubclient

During research for this project, i found some projects working on ths promissing concept.

initialy found
https://github.com/GeoscienceAustralia/gnss-mqtt

and
http://www.ignss2018.unsw.edu.au/sites/ignss2018/files/u80/Slides/D2-S3-ThC-Wang.pdf

and comercial
https://esprtk.com/

just found:
https://github.com/geerdkakes/mqtt-rtk

it would be nice if mqtt could be implemented to
https://github.com/nebkat/esp32-xbee 


# Compile:

require #include "PubSubClient.h" library https://github.com/knolleary/pubsubclient.git PlatforIO ID_89

# see platformio.ini:

require #include "verysimpletimer.h"  https://github.com/hagre/VerySimpleTimer_Library.git
require #include "SyncWifiConnectionESP32.h"  https://github.com/hagre/SyncWiFIConnectionESP32_Library.git
require #include "SyncMQTTConnectionESP32.h" https://github.com/hagre/SyncMQTTConnectionESP32_Library.git
require #include "rtcmstreamsplitter.h" https://github.com/hagre/RTCM_Stream_Splitter_Library.git


ToDo:
* Find a good way to update of root_ca 
* Find a better and futureproof name for this new RTCM transmitting and hosting standard!
* Complete the design of using the MQTT broker as an correction data server according to an NTRIP Caster

by hagre 
06 2019 - 2020 
