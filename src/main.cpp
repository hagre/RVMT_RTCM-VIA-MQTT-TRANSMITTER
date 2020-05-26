/*
INFO: This Software is made for my private use. I like to share, but for further details read the LICENCE file!

RVMT (RTCM VIA MQTT TRANSMITTER) is using the MQTT protocol (as a secure and opensource alternative to NTRIP) to get RTK correction data for my rover GPS units.

ESP32 firmwares 2 in 1.
Basic selection via #define in the top of this code

by hagre 
06 2019 - 2020 
V2.1 - check LOG file
*/

//Select Type of equipment
#define SERVER_ROOF_NOTE
//#define CLIENT_ROVER_NOTE
#define USE_MY_SECRETS //self explaining (more or less just for me to use)

//ONLY FOR INFO: SELECTION OF COMPLETE CONFIGURATION POSSIBILITIES
//#define SERVER_ROOF_NOTE
//#define CLIENT_ROVER_NOTE
//#define USE_MY_SECRETS //self explaining (more or less just for me to use)
//#define DEBUG_UART_ENABLED //enable or disable with // in front of #define     //self explaining (more or less just for me to use)
//#define ESP32_NODE
//#define USB_CONNECTED_NODE // has USB connection to PC
//#define WIFI_CONNECTED_NODE //enable or disable with // in front of #define
//or
//#define ETHERNET_CONNECTED_NODE //enable or disable with // in front of #define
//#define MQTT_VIA_SECURE_WIFI_NODE //enable or disable with // in front of #define 
//#define MQTT_VIA_NOT_SECURE_WIFI_NODE //enable or disable with // in front of #define 
//#define RTCM_VIA_MQTT_TRANSMITTING_WITH_PASSWORD //enable or disable with // in front of #define 

#ifdef SERVER_ROOF_NOTE
  #define DEBUG_UART_ENABLED //enable or disable with // in front of #define     //self explaining (more or less just for me to use)
  //Features
  #define ESP32_NODE
  #define USB_CONNECTED_NODE 
  #define WIFI_CONNECTED_NODE 
  #define LAN_IP_RANGE 20 //config as required
  #define MQTT_VIA_SECURE_WIFI_NODE
  //#define MQTT_VIA_NOT_SECURE_WIFI_NODE
  #define RTCM_VIA_MQTT_TRANSMITTING_WITH_PASSWORD

  //Settings
  #ifdef USE_MY_SECRETS
    #include "secrets/secrets_server.h"
  #endif
  #define YOUR_WIFI_HOSTNAME "RTCM_MQTT_SENDER"
  #define IP_1_THIS_NODE 192
  #define IP_2_THIS_NODE 168
  #define IP_3_THIS_NODE LAN_IP_RANGE
  #define IP_4_THIS_NODE 115
  #define IP_1_GATEWAY 192
  #define IP_2_GATEWAY 168
  #define IP_3_GATEWAY LAN_IP_RANGE  
  #define IP_4_GATEWAY 1
#endif

#ifdef CLIENT_ROVER_NOTE
  #define DEBUG_UART_ENABLED //enable or disable with // in front of #define     //self explaining (more or less just for me to use)
  //Features
  #define ESP32_NODE
  #define USB_CONNECTED_NODE
  #define WIFI_CONNECTED_NODE
  #define LAN_IP_RANGE 20 //config as required
  #define MQTT_VIA_SECURE_WIFI_NODE
  //#define MQTT_VIA_NOT_SECURE_WIFI_NODE
  #define RTCM_VIA_MQTT_TRANSMITTING_WITH_PASSWORD 

  //Settings
  #ifdef USE_MY_SECRETS
    #include "secrets/secrets_client.h"
  #endif
  #define YOUR_WIFI_HOSTNAME "RTCM_MQTT_RECEIVER"
  #define IP_1_THIS_NODE 192
  #define IP_2_THIS_NODE 168
  #define IP_3_THIS_NODE LAN_IP_RANGE
  #define IP_4_THIS_NODE 127
  #define IP_1_GATEWAY 192
  #define IP_2_GATEWAY 168
  #define IP_3_GATEWAY LAN_IP_RANGE
  #define IP_4_GATEWAY 1
#endif

#ifdef WIFI_CONNECTED_NODE 
  #ifndef YOUR_WIFI_SSID
    #define YOUR_WIFI_SSID "WLAN"
  #endif
  #ifndef YOUR_WIFI_PASSWORD
    #define YOUR_WIFI_PASSWORD "Password"
  #endif
  #define WIFI_WAIT_FOR_CONNECTION 10000 //ms time
  #define WIFI_WAIT_FOR_RECONNECTION 5000 //ms time
  #ifndef YOUR_WIFI_HOSTNAME
    #define YOUR_WIFI_HOSTNAME "RTCM_MQTT_NOTE"
  #endif 
#endif 

#if defined(WIFI_CONNECTED_NODE) || defined (ETHERNET_CONNECTED_NODE)
  #ifndef IP_MQTT_BROKER
    #define IP_MQTT_BROKER "test.mosquitto.com" //config as required 
  #endif
  #ifndef MQTT_CLIENT_ID_FOR_BROKER
    #define MQTT_CLIENT_ID_FOR_BROKER "TestClient" //config as required 
  #endif
  #define MQTT_VERSION 4 //4 // MQTT_VERSION_3_1  3, MQTT_VERSION_3_1_1 

  #ifdef RTCM_VIA_MQTT_TRANSMITTING_WITH_PASSWORD
    #ifndef MQTT_BROKER_USERNAME
      #define MQTT_BROKER_USERNAME "MQTTuser" //config as required 
    #endif
    #ifndef MQTT_BROKER_PASSWORD
      #define MQTT_BROKER_PASSWORD "MQTTpassword" //config as required 
    #endif
  #endif 
  #ifdef MQTT_VIA_SECURE_WIFI_NODE
    #define MQTT_CONNECTION_PORT 8883 //8882 = not secure on LAN only // 8883 TLS //config as required
  #endif
  #ifdef MQTT_VIA_NOT_SECURE_WIFI_NODE
    #define MQTT_CONNECTION_PORT 8882 //8882 = not secure on LAN only // 8883 TLS //config as required
  #endif
  #define MQTT_SET_KEEPALIVE 15 //15 = 15sec
  #define MQTT_SET_SOCKET_TIMEOUT 10 // 10sec
  #define MQTT_WAIT_FOR_SERVER_CONNECTION 30000 // 10000ms //config as required
  #define MQTT_WAIT_FOR_SERVER_RECONNECTION 15000 // 10000ms //config as required
  #define MQTT_WAIT_BETWEEN_SENDING_MSG 50 // 30ms //config as required
  #define MQTT_MAX_TIME_FOR_RESEND_MSG 1000 // 10000ms //config as required
  #define MQTT_MSG_RETAINED false // false //config as required
  #ifndef MQTT_RTCM_TOPIC_COMMAND
    #define MQTT_RTCM_TOPIC_COMMAND "/NTRIP/Base/XYZ01/Commands/" //config as required 
  #endif
  #ifndef MQTT_RTCM_TOPIC
    #define MQTT_RTCM_TOPIC "/NTRIP/Base/XYZ01/RTCM/" //config as required 
  #endif
  #define MQTT_BUFFER_SIZE 1024
  #define MQTT_MAX_PACKET_SIZE MQTT_BUFFER_SIZE

  #define RTCM_1005_DATA_INTERVALL 20000 //20000
  #define MSM_TYPE 4 // 4 or 7 (high resolution)
  #define RTCM_VIA_MQTT_UART_TRANSMITTING_BOUD 230400 //config as required 
  #define RTCM_UART_HARDWARE_PORT 2 //config as required //set 2, USB == 0, 1 == UART1 F9P, 2 == UART2 F9P rewire rquired not fitting on ardusimple
  #define RTCM_BUFFER_SIZE 1024 //config as required
  #define RTCM_LOOP_BUFFER_SIZE 6 //config as required // •  RTCM 1005 Stationary RTK reference station ARP•  RTCM 1074 GPS MSM4•  RTCM 1084 GLONASS MSM4•  RTCM 1094 Galileo MSM4•  RTCM 1124 BeiDou MSM4•  RTCM 1230 GLONASS code-phase biases
#endif

#ifdef USB_CONNECTED_NODE 
  #ifdef DEBUG_UART_ENABLED 
    #define DEBUG_UART_HARDWARE_PORT 0 //config as required // USB == 0
    #define DEBUG_UART_BOUD 230400 //config as required 
    #define DEBUG_WAITING_TIME 1000 //config as required 
  #endif
#endif 
// -------------------------------- End of Config -----------------------------------------------

// -------------------------------- Begin of Program --------------------------------------------
//include libraries
#include <Arduino.h>
#include "mysimpletimer.h"

//sanitycheck
#if defined(SERVER_ROOF_NOTE) && defined (CLIENT_ROVER_NOTE)
  #error "ERROR: select just one basic purpose of this code"
#endif
#if !defined(SERVER_ROOF_NOTE) && !defined(CLIENT_ROVER_NOTE)
  #error "ERROR: please decide for one basic purpose of this code"
#endif

#ifdef WIFI_CONNECTED_NODE
  #define LAN_CONNECTED_NODE
  #ifdef ETHERNET_CONNECTED_NODE
    #error "ERROR: select one: WIFI or ETHERNET"
  #endif
#endif

#ifdef ETHERNET_CONNECTED_NODE
    #define LAN_CONNECTED_NODE
  #ifdef WIFI_CONNECTED_NODE
    #error "ERROR: select one: WIFI or ETHERNET"
  #endif
#endif

#ifdef DEBUG_UART_ENABLED
  uint16_t debug_count = 0;
  MySimpleTimer DebugLoopTimer;
  #ifndef USB_CONNECTED_NODE
    #error "DEBUG_UART_ENABLED needs USB_CONNECTED_NODE to be configured"
  #endif
#endif 

#ifndef LAN_CONNECTED_NODE
  #error "ERROR: select at least one: WIFI or ETHERNET"
#endif

//include libraries
#include <Arduino.h>
#include "mysimpletimer.h"

#ifdef ESP32_NODE
  #define UART0RX 3 //ESP32 GPIOs
  #define UART0TX 1
  #define UART1RX 16 //26 //ESP32 look like UNO - modified for Ardusimpleconnection 
  #define UART1TX 17 //12 problem SPI internal //ESP32 look like UNO - modified for Ardusimpleconnection 
  #define UART2RX 13 //16 //ESP32 look like UNO - modified for Ardusimpleconnection 
  #define UART2TX 26 //17 //ESP32 look like UNO - modified for Ardusimpleconnection 
#endif

#ifdef USB_CONNECTED_NODE
  #ifdef DEBUG_UART_ENABLED
    #include <HardwareSerial.h>
    uint32_t serialDebugPortBoud = DEBUG_UART_BOUD;
    #if DEBUG_UART_HARDWARE_PORT == 0  
      #define DEBUG_UART_RX UART0RX
      #define DEBUG_UART_TX UART0TX
    #elif DEBUG_UART_HARDWARE_PORT == 1  
      #define DEBUG_UART_RX UART1RX
      #define DEBUG_UART_TX UART1TX
    #elif DEBUG_UART_HARDWARE_PORT == 2 
      #define DEBUG_UART_RX UART2RX
      #define DEBUG_UART_TX UART2TX 
    #endif
    HardwareSerial SerialDebug(DEBUG_UART_HARDWARE_PORT);
  #endif
#endif

#ifdef LAN_CONNECTED_NODE  
  struct RTCMTransmitBuffer_t {
    bool readyToSend;
    bool sending;
    bool alreadySent;
    unsigned long nrOfInternalEpoche;
    unsigned millisTimeOfReceive;
    int16_t typeOfRTCMMsg;
    byte RTCMMsg [RTCM_BUFFER_SIZE]; 
    uint16_t msgLength;
  } rTCMTransmitLoopBuffer [RTCM_LOOP_BUFFER_SIZE];

  unsigned long rTCMTransmitLoopBufferRXEpoch = 0;
  unsigned long rTCMTransmitLoopBufferTXEpoch = 0;

  uint8_t serialRTCMtoMQTTBuffer[MQTT_BUFFER_SIZE];

  bool rTCMviaMQTTisActive = true;

  int8_t LANStatus = -5; //Connected to Network //WIFI or ETHERNET //-5 = init, -2 = just disconnected, -1 = wait to reconnect, 0 = disconnected, 1 = connecting, 2 = just connected,  3 = still connected

  const char* mQTTBrokerHostName = IP_MQTT_BROKER;

  #ifdef RTCM_VIA_MQTT_TRANSMITTING_WITH_PASSWORD
    const char * mqttUsername = MQTT_BROKER_USERNAME;
    const char * mqttPassword = MQTT_BROKER_PASSWORD;
  #endif

  const IPAddress Node_IP(IP_1_THIS_NODE, IP_2_THIS_NODE, IP_3_THIS_NODE, IP_4_THIS_NODE);
  const IPAddress gateway(IP_1_GATEWAY, IP_2_GATEWAY, IP_3_GATEWAY, IP_4_GATEWAY);
  const IPAddress subnet(255, 255, 255, 0);
  //IPAddress mQTTBrokerIP;

  #ifdef WIFI_CONNECTED_NODE
    #include <WiFi.h>
    const char* ssid     = YOUR_WIFI_SSID; 
    const char* password = YOUR_WIFI_PASSWORD;
    MySimpleTimer WIFIWaitForConnectionTimer;
    MySimpleTimer WIFIWaitForReconnectingTimer;

    #ifdef MQTT_VIA_SECURE_WIFI_NODE
      #include <WiFiClientSecure.h>
      WiFiClientSecure mqttLANClient;
      #ifndef MQTT_BROKER_CA_CERT 
        //This is an unvalide root_CA_cert just to show you the format
        #include "root_ca.h"
      #endif
    #endif  

    #ifdef MQTT_VIA_NOT_SECURE_WIFI_NODE
      #include <WiFiClient.h>
      WiFiClient mqttLANClient;
    #endif

    #include <PubSubClient.h>
    PubSubClient mqttPubSubClient(mqttLANClient);
    const char * mqttPubSubClientId = MQTT_CLIENT_ID_FOR_BROKER;
  #endif

  #ifdef ETHERNET_CONNECTED_NODE
    // LAN ToDo... ------------------------------------------------------------------------------------------------ToDO-----------------------------------------------
  #endif
  
  #include <HardwareSerial.h>
  uint32_t serialRTCMPortBoud = RTCM_VIA_MQTT_UART_TRANSMITTING_BOUD;
  #if RTCM_UART_HARDWARE_PORT == 0  
    #define RTCM_UART_RX UART0RX
    #define RTCM_UART_TX UART0TX
  #elif RTCM_UART_HARDWARE_PORT == 1  
    #define RTCM_UART_RX UART1RX
    #define RTCM_UART_TX UART1TX
  #elif RTCM_UART_HARDWARE_PORT == 2 
    #define RTCM_UART_RX UART2RX
    #define RTCM_UART_TX UART2TX 
  #endif
  HardwareSerial SerialRTCM(RTCM_UART_HARDWARE_PORT);
  MySimpleTimer MQTTWaitForConnectionTimer;
  MySimpleTimer MQTTWaitForReconnectionTimer;
  MySimpleTimer MQTTWaitBetweenSendingMsg;
  MySimpleTimer RTCMTransmit1005Timer;
  int8_t MQTTStatus = -5; //Connected to MQTT broker // -5 init, -3 LAN just disconnected, -2 just disconnected, -1 wait to reconnect, 0 disconnected, 1 connecting, 2 just connected, 3 subscribing, 4 subscribed and connected
  #include <rtcmstreaminput.h>
  RTCMStreamInput RTCMStream;    
#endif

void receivedMQTTCallback(char* topic, byte* payload, unsigned int length) {
  #ifdef SERVER_ROOF_NOTE
    if (((char)payload[0]) == 'O'){ //ON
      if (((char)payload[1]) == 'N'){
        rTCMviaMQTTisActive = true;
        #ifdef DEBUG_UART_ENABLED
          SerialDebug.println("Message received: ON NNNNNNNNNNNNNNNNNNN");
        #endif
      }
    }
    if (((char)payload[0]) == 'O'){ //OFF
      if (((char)payload[1]) == 'F'){
        //if (((char)payload[2]) == 'F'){
          rTCMviaMQTTisActive = false;
          #ifdef DEBUG_UART_ENABLED
            SerialDebug.println("Message received: OFF FFFFFFFFFFFFFFFFF");
          #endif
        //}
      }
    }
  #endif

  #ifdef CLIENT_ROVER_NOTE
    for (int i = 0; i < length; i++) {
      SerialRTCM.write(payload[i]);
    }
  #endif
  
  #ifdef DEBUG_UART_ENABLED 
    
    SerialDebug.print("Message received: ");
    SerialDebug.println(topic);
    SerialDebug.print("payload: ");
    for (int i = 0; i < length; i++) {
      SerialDebug.print((char)payload[i]);
      SerialDebug.print(" ");
    }
    SerialDebug.print(" // ");
    for (int i = 0; i < length; i++) {
      SerialDebug.print(payload[i]);
      SerialDebug.print(" ");
    }
    SerialDebug.println();
    
  #endif
}

void setup() { // -------------------------------- S E T U P --------------------------------------------

  #ifdef DEBUG_UART_ENABLED
    SerialDebug.begin (serialDebugPortBoud, SERIAL_8N1, DEBUG_UART_RX, DEBUG_UART_TX); //Debug output, usually USB Port
    SerialDebug.println ("Setup");
    DebugLoopTimer.setIntervalMs (DEBUG_WAITING_TIME);
  #endif

  #ifdef LAN_CONNECTED_NODE
    #ifdef WIFI_CONNECTED_NODE 
      WiFi.mode(WIFI_STA);
      WiFi.config(Node_IP, gateway, subnet, gateway);// primDNS, secDNS);
      WiFi.setHostname(YOUR_WIFI_HOSTNAME);
      WiFi.begin(YOUR_WIFI_SSID, YOUR_WIFI_PASSWORD);
      #ifdef DEBUG_UART_ENABLED
        SerialDebug.println("WIFI configured");
        SerialDebug.println(WiFi.localIP());
        SerialDebug.println(WiFi.getHostname());
        //SerialDebug.println(WiFi.getAutoReconnect());
        SerialDebug.println(WiFi.status());
      #endif  
      WIFIWaitForConnectionTimer.setIntervalMs (WIFI_WAIT_FOR_CONNECTION); 
      WIFIWaitForReconnectingTimer.setIntervalMs (WIFI_WAIT_FOR_RECONNECTION); 
    #endif 
    #ifdef ETHERNET_CONNECTED_NODE
      #ifdef DEBUG_UART_ENABLED
        SerialDebug.println ("Starting ETERNET modul");
      #endif 
      //init ETHERNET Modul and connection
      //todo ...
    #endif
  #endif

  SerialRTCM.begin (serialRTCMPortBoud, SERIAL_8N1, RTCM_UART_RX, RTCM_UART_TX); //RTCM IN and output //F9P input/output
  SerialRTCM.setRxBufferSize (RTCM_BUFFER_SIZE);
  #ifdef MQTT_VIA_SECURE_WIFI_NODE
    // set TLS certificate
    mqttLANClient.setCACert(root_CA_Cert);
  #endif

  #ifdef MQTT_VIA_NOT_SECURE_WIFI_NODE
    //Nothing todo
  #endif

  mqttPubSubClient.setClient(mqttLANClient);
  mqttPubSubClient.setServer(IP_MQTT_BROKER, MQTT_CONNECTION_PORT);
  mqttPubSubClient.setCallback(receivedMQTTCallback);
  mqttPubSubClient.setBufferSize(MQTT_MAX_PACKET_SIZE);
  #ifdef DEBUG_UART_ENABLED
    SerialDebug.print ("MQTT Buffersize: ");
    SerialDebug.println (mqttPubSubClient.getBufferSize ());
  #endif
  mqttPubSubClient.setKeepAlive(MQTT_SET_KEEPALIVE);
  mqttPubSubClient.setSocketTimeout(MQTT_SET_SOCKET_TIMEOUT);

  MQTTWaitForConnectionTimer.setIntervalMs (MQTT_WAIT_FOR_SERVER_CONNECTION);
  MQTTWaitForReconnectionTimer.setIntervalMs (MQTT_WAIT_FOR_SERVER_RECONNECTION);

  #ifdef SERVER_ROOF_NOTE
    RTCMTransmit1005Timer.setIntervalMs (RTCM_1005_DATA_INTERVALL);
    MQTTWaitBetweenSendingMsg.setIntervalMs (MQTT_WAIT_BETWEEN_SENDING_MSG);

    // initiate Loop buffer variable
    #if MSM_TYPE == 4
      rTCMTransmitLoopBuffer[0].typeOfRTCMMsg = 1074; //RTCM 1074 GPS MSM4
      rTCMTransmitLoopBuffer[1].typeOfRTCMMsg = 1084; //RTCM 1084 GLONASS MSM4
      rTCMTransmitLoopBuffer[2].typeOfRTCMMsg = 1094; //RTCM 1094 Galileo MSM4
      rTCMTransmitLoopBuffer[3].typeOfRTCMMsg = 1124; //RTCM 1124 BeiDou MSM4
      rTCMTransmitLoopBuffer[4].typeOfRTCMMsg = 1230; //RTCM 1230 GLONASS code-phase biases
      rTCMTransmitLoopBuffer[5].typeOfRTCMMsg = 1005; //RTCM 1005 Stationary RTK reference station ARP
    #endif
    #if MSM_TYPE == 7
      rTCMTransmitLoopBuffer[0].typeOfRTCMMsg = 1077; //RTCM 1077 GPS MSM4
      rTCMTransmitLoopBuffer[1].typeOfRTCMMsg = 1087; //RTCM 1087 GLONASS MSM4
      rTCMTransmitLoopBuffer[2].typeOfRTCMMsg = 1097; //RTCM 1097 Galileo MSM4
      rTCMTransmitLoopBuffer[3].typeOfRTCMMsg = 1127; //RTCM 1127 BeiDou MSM4
      rTCMTransmitLoopBuffer[4].typeOfRTCMMsg = 1230; //RTCM 1230 GLONASS code-phase biases
      rTCMTransmitLoopBuffer[5].typeOfRTCMMsg = 1005; //RTCM 1005 Stationary RTK reference station ARP
    #endif
    for (int i = 0; i < RTCM_LOOP_BUFFER_SIZE; i++){
      rTCMTransmitLoopBuffer[i].alreadySent = true; //to start sending of first msg
      rTCMTransmitLoopBuffer[i].readyToSend = false;
      rTCMTransmitLoopBuffer[i].sending = false;
    }
  #endif

  #ifdef CLIENT_ROVER_NOTE
    //ToDo+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  #endif  
}

void loop() { // -------------------------------- L O O P --------------------------------------------
  #ifdef DEBUG_UART_ENABLED
    //SerialDebug.println ("LOOP");
  #endif

  #ifdef SERVER_ROOF_NOTE
    //SerialRead Input
    uint16_t nrReceivedSerial = SerialRTCM.available(); //count serial incomming data 
    if (nrReceivedSerial > RTCM_BUFFER_SIZE){ // Check Buffersize, and read just the first sample
      nrReceivedSerial = RTCM_BUFFER_SIZE;
      #ifdef DEBUG_UART_ENABLED 
        SerialDebug.println (" Buffersize Serial to low ");  
      #endif
    }
    
    #ifdef DEBUG_UART_ENABLED 
      if (nrReceivedSerial > 0 ){
        SerialDebug.print (" RTCM Serial received ");
        SerialDebug.println (nrReceivedSerial);
      }
    #endif

    for (uint16_t i = 0; i < nrReceivedSerial; i++){
      int16_t typeOfMsg; 
      int16_t newMsgChar;
      newMsgChar = SerialRTCM.read();

      typeOfMsg = RTCMStream.inputByte (newMsgChar);
      if (typeOfMsg > 0) { // new msg completly arrived
        #ifdef DEBUG_UART_ENABLED 
          SerialDebug.print (" Type of RTCM ");
          SerialDebug.println (typeOfMsg);
        #endif

        unsigned long millisOfInput = millis();
        for (int x = 0; x < RTCM_LOOP_BUFFER_SIZE; x++){
          if (typeOfMsg == rTCMTransmitLoopBuffer[x].typeOfRTCMMsg){
            if (i == 0) {
              rTCMTransmitLoopBufferRXEpoch++;
              #ifdef DEBUG_UART_ENABLED 
                SerialDebug.print (" New epoch is ");
                SerialDebug.println (rTCMTransmitLoopBufferRXEpoch);
                for (int y = 0; y < RTCM_LOOP_BUFFER_SIZE; y++){
                  if (!rTCMTransmitLoopBuffer[y].alreadySent){ // all msg of this epoch alreadySent
                    #ifdef DEBUG_UART_ENABLED 
                      SerialDebug.print (" MQTT transmitting to slow for ");
                      SerialDebug.println (y);
                    #endif
                  }
                }
              #endif
            }
            if (rTCMTransmitLoopBuffer[x].alreadySent){ //free to use

              #ifdef DEBUG_UART_ENABLED 
                //SerialDebug.print (" Time since last saving this type of msg ");
                //SerialDebug.println (millisOfInput - rTCMTransmitLoopBuffer[i].millisTimeOfReceive);
              #endif

              rTCMTransmitLoopBuffer[x].millisTimeOfReceive = millisOfInput;
              rTCMTransmitLoopBuffer[x].typeOfRTCMMsg = typeOfMsg;
              for (uint16_t z = 0; z < RTCMStream.outputStreamLength; z++){
                rTCMTransmitLoopBuffer[x].RTCMMsg[z] = RTCMStream.outputStream[z];
                #ifdef DEBUG_UART_ENABLED 
                  //SerialDebug.print (rTCMTransmitLoopBuffer[i].RTCMMsg, HEX); 
                #endif
              }
              rTCMTransmitLoopBuffer[x].msgLength = RTCMStream.outputStreamLength;
              rTCMTransmitLoopBuffer[x].nrOfInternalEpoche = rTCMTransmitLoopBufferRXEpoch;
              rTCMTransmitLoopBuffer[x].sending = false;
              rTCMTransmitLoopBuffer[x].alreadySent = false;
              rTCMTransmitLoopBuffer[x].readyToSend = true; 
            }
            else { //This Msg buffer still not complete transmitted
              #ifdef DEBUG_UART_ENABLED 
                if (rTCMTransmitLoopBuffer[x].sending){ //not free to use
                  SerialDebug.println (" RTCM Msg Buffer blocked - still sending");
                }
                else {
                  SerialDebug.println (" RTCM Msg Buffer blocked - sending not marked as finished");
                }
              #endif
            }
          }
        }
      #ifdef DEBUG_UART_ENABLED
        //SerialDebug.println (" Break after recognized Msg ");
      #endif
      goto end_nrReceivedSerial_for_loop; //break; is not working correct //to extit serial read after one msg and continue loop, next time read further..
      }
    }
    end_nrReceivedSerial_for_loop:
    #ifdef DEBUG_UART_ENABLED 
      //SerialDebug.print (" EXIT_RX ");
    #endif
  #endif  

  #ifdef CLIENT_ROVER_NOTE
    //ToDo+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  #endif

  #ifdef LAN_CONNECTED_NODE
    //WIFI CONNECTION MANAGER
    #ifdef WIFI_CONNECTED_NODE
      int8_t WIFIStatus = WiFi.status();
      if (LANStatus == -5){ //First Loop after boot
        #ifdef DEBUG_UART_ENABLED
          SerialDebug.println (" First loop after boot");
        #endif
        WIFIWaitForConnectionTimer.resetTimingNow (millis());
        LANStatus = 0; //disconnected
      }
      else if (LANStatus == -2){ //First Loop after WIFI just disconnected
        WIFIWaitForReconnectingTimer.resetTimingNow (millis());
        LANStatus = -1; //wait to reconnect
        #ifdef DEBUG_UART_ENABLED
          SerialDebug.println (" First Loop after WIFI just disconnected");
        #endif
      }
      else if (LANStatus == -1){ //wait to reconnect still disconnected
        if (WIFIWaitForReconnectingTimer.getStatus(millis()) >= 0){ //Chech if ready for reconnect
          LANStatus = 0;  //disconnected
          #ifdef DEBUG_UART_ENABLED
            SerialDebug.println (" Timer elapsed - time to connect again");
          #endif
        }
      }
      else if (LANStatus == 0){ //disconnected and time to connect again
        WiFi.begin(YOUR_WIFI_SSID, YOUR_WIFI_PASSWORD);
        WIFIWaitForConnectionTimer.resetTimingNow (millis()); //Start new timing for NEW connection
        LANStatus = 1; //connecting
        #ifdef DEBUG_UART_ENABLED
          SerialDebug.println (" Starting WIFI connection");
        #endif
      }
      else if (LANStatus == 1){ //connecting
        if (WIFIStatus == WL_CONNECTED){ //WIFI connected
          LANStatus = 2; //connected to LAN
          #ifdef DEBUG_UART_ENABLED
            SerialDebug.println (" Just WIFI Connected");
          #endif  
        }
        else if (WIFIWaitForConnectionTimer.getStatus(millis()) >= 0){ //Chech if try to connect takes too long
          WiFi.disconnect();
          LANStatus = -2; //just disconnected
          #ifdef DEBUG_UART_ENABLED
            SerialDebug.println (" It took to long => WIFI Disconnect");
          #endif
        }
        else {
          #ifdef DEBUG_UART_ENABLED
            //SerialDebug.print (".");
          #endif
        }
      }
      else if (LANStatus == 2){ //just connected
        LANStatus = 3; //still connected to LAN
        #ifdef DEBUG_UART_ENABLED
          SerialDebug.println (" Again, this important info: WIFI connected");
        #endif 
      }
      else if (LANStatus == 3){ //still connected
        #ifdef DEBUG_UART_ENABLED
          //SerialDebug.println ("WIFI still connected");
        #endif
        if (WIFIStatus != WL_CONNECTED){ //WIFI NOT connected anymore
          WiFi.disconnect();
          LANStatus = -2; //just disconnected
          #ifdef DEBUG_UART_ENABLED
            SerialDebug.println (" WIFI connection lost!");
          #endif
        }
      }
    #endif

    //LAN CONNECTION MANAGER
    #ifdef ETHERNET_CONNECTED_NODE
      //init ETHERNET Modul and connection
      //todo ...
      LANStatus = -2;
    #endif
  #endif
 
  //RTCM_VIA_MQTT_NODE
  if (LANStatus == -2){ // LAN is just disconnected
    if (MQTTStatus > -2){ // MQTT better than disconnected and LAN just disconnected 
      MQTTStatus = -3;
      #ifdef DEBUG_UART_ENABLED 
        SerialDebug.println (" LAN for MQTT just disconnected"); //disconnect -> MQTT stop ");
      #endif
    }
  }
  else if (LANStatus == 3){ // LAN is connected look for more
    if (MQTTStatus == -5){ // MQTT init
      MQTTStatus = 0; //MQTT disconnected 
      #ifdef DEBUG_UART_ENABLED 
        SerialDebug.println(" MQTT init loop");
      #endif
    }
    else if (MQTTStatus == -3){ // LAN just disconnected
      mqttPubSubClient.disconnect(); 
      MQTTStatus = -2; //MQTT just disconnected
      #ifdef DEBUG_UART_ENABLED 
        SerialDebug.println(" MQTT stop - LAN connection lost");
      #endif
    }
    else if (MQTTStatus == -2){ // MQTT just disconnected
      MQTTWaitForReconnectionTimer.resetTimingNow (millis());
      MQTTStatus = -1; //MQTT wait to reconnect
      #ifdef DEBUG_UART_ENABLED 
        SerialDebug.println(" MQTT just disconnected");
      #endif
    }
    else if (MQTTStatus == -1){ //MQTT wait to reconnect
      if (MQTTWaitForReconnectionTimer.getStatus (millis()) >= 0){
        MQTTStatus = 0; //MQTT disconncted
        #ifdef DEBUG_UART_ENABLED 
          SerialDebug.println (" MQTT time to reconnect ");
        #endif
      }
    }
    else if (MQTTStatus == 0){ //MQTT disconnected 
      MQTTWaitForConnectionTimer.resetTimingNow (millis());
      #ifdef RTCM_VIA_MQTT_TRANSMITTING_WITH_PASSWORD
        #ifdef DEBUG_UART_ENABLED 
          SerialDebug.println(" Connecting MQTT with PW");
        #endif
        mqttPubSubClient.connect(mqttPubSubClientId, mqttUsername, mqttPassword); 
      #else
        #ifdef DEBUG_UART_ENABLED 
          SerialDebug.println(" Connecting MQTT without PW");
        #endif
        mqttPubSubClient.connect(mqttPubSubClientId);
      #endif
      MQTTStatus = 1; //MQTT connecting
    }
    else if (MQTTStatus == 1){
      if (mqttPubSubClient.connected ()){ 
        MQTTStatus = 2; //MQTT just connected
        #ifdef DEBUG_UART_ENABLED 
          SerialDebug.println(" MQTT just connected");
        #endif
      }
      else if (MQTTWaitForConnectionTimer.getStatus (millis()) >= 0){
        MQTTStatus = -2; // MQTT just disconnected
      }
    }
    else if (MQTTStatus == 2){ //MQTT just connected
      MQTTStatus = 3; //subscribing
      if (!mqttPubSubClient.loop()){
        MQTTStatus = -2; // MQTT just disconnected
        #ifdef DEBUG_UART_ENABLED 
          SerialDebug.println(" MQTT connecting problem in Loop");
        #endif
      }
    }
    else if (MQTTStatus == 3){ //subscribing
      if (!mqttPubSubClient.loop()){
        MQTTStatus = -2; // MQTT just disconnected
        #ifdef DEBUG_UART_ENABLED 
          SerialDebug.println(" MQTT connecting problem in Loop during subscribing");
        #endif
      }
      uint8_t tempStatus = 0;
      uint8_t requestetedtempStatus = 0;
      if (mqttPubSubClient.subscribe(MQTT_RTCM_TOPIC_COMMAND)){
        tempStatus++;
      }
      #ifdef SERVER_ROOF_NOTE
        requestetedtempStatus = 1; // to test if more than one (if required) topics is subscribed 
      #endif
      #ifdef CLIENT_ROVER_NOTE
        if (mqttPubSubClient.subscribe(MQTT_RTCM_TOPIC)){
          tempStatus++;
        }
        requestetedtempStatus = 2;
      #endif
      if (tempStatus == requestetedtempStatus){
        MQTTStatus = 4; //subscribed and connected
        #ifdef DEBUG_UART_ENABLED 
          SerialDebug.println (" Subscribed to all topics ");
        #endif
      }
    }
    else if (MQTTStatus == 4){//subscribed and connected
      if (!mqttPubSubClient.loop()){
        MQTTStatus = -2; // MQTT just disconnected
        #ifdef DEBUG_UART_ENABLED 
          SerialDebug.println(" MQTT connecting problem in during normal LOOP");
        #endif
      }

      #ifdef DEBUG_UART_ENABLED 
        //SerialDebug.println ("MQTT client connected to broker ");
        debug_count++;
        unsigned long debugTimenow = millis ();     
        if (DebugLoopTimer.getStatus (debugTimenow) >= 0){
          //SerialDebug.println (" MQTT testpub ... TTTTTTTTTTTT");
          //mqttPubSubClient.publish ("/Connection/", "YES");
          SerialDebug.print (" Loops: ");
          SerialDebug.println (debug_count);
          debug_count = 0;
        }
      #endif

      #ifdef DEBUG_UART_ENABLED 
        //SerialDebug.println ("MQTT Loop duty ");
      #endif

      //RECEIVE // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++RECEIVE++++++++++++++++++++++++
      // in Callback function

      //TRANSMIT // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++TRANSMIT++++++++++++++++++++++++  
      #ifdef SERVER_ROOF_NOTE 
        unsigned long millisOfOutput = millis();
        #ifdef DEBUG_UART_ENABLED 
          //SerialDebug.println (" Ready to send next MQTT - time elapsed ");
          //SerialDebug.println (" LOOP, ");
        #endif
        if (MQTTWaitBetweenSendingMsg.getStatus (millisOfOutput) >= 0){ // wait between sending of MQTT Msg to avoid overflow/losses
          #ifdef DEBUG_UART_ENABLED 
            SerialDebug.println (" Ready to send, ");
          #endif
          for (int i = 0; i < RTCM_LOOP_BUFFER_SIZE; i++){// check complete buffer
            #ifdef DEBUG_UART_ENABLED 
              SerialDebug.print (rTCMTransmitLoopBuffer[i].readyToSend);
            #endif
            if (rTCMTransmitLoopBuffer[i].readyToSend){
              #ifdef DEBUG_UART_ENABLED 
                    //SerialDebug.print ("Ready to send next MQTT found  ");
              #endif
              rTCMTransmitLoopBuffer[i].sending = true; // block variable
              rTCMTransmitLoopBuffer[i].readyToSend = false;
              if (i == 0) {
                #ifdef DEBUG_UART_ENABLED 
                  if (rTCMTransmitLoopBuffer[i].nrOfInternalEpoche - rTCMTransmitLoopBufferTXEpoch > 1){
                    SerialDebug.print (" Missed epoch! ");
                    SerialDebug.println (rTCMTransmitLoopBuffer[i].nrOfInternalEpoche - rTCMTransmitLoopBufferTXEpoch);
                  }
                #endif
                rTCMTransmitLoopBufferTXEpoch = rTCMTransmitLoopBuffer[i].nrOfInternalEpoche;
              }
              // preperation to transmitt 
              String MQTTMsgTopic = "";
              String MQTTMsgType = String (rTCMTransmitLoopBuffer[i].typeOfRTCMMsg);
              MQTTMsgTopic = MQTT_RTCM_TOPIC + MQTTMsgType + "/";

              byte bMQTTMsg [RTCM_BUFFER_SIZE];
              byte* pbMQTTMsg = bMQTTMsg;
              #ifdef DEBUG_UART_ENABLED 
                //SerialDebug.print (" OutputStreamLength ");
                //SerialDebug.println (RTCMStream.outputStreamLength);
              #endif

              for (uint16_t x = 0; x < rTCMTransmitLoopBuffer[i].msgLength; x++){
                bMQTTMsg [x]= rTCMTransmitLoopBuffer[i].RTCMMsg[x];
                #ifdef DEBUG_UART_ENABLED 
                  //SerialDebug.print (bMQTTMsg[x], HEX); 
                #endif
              }

              int16_t check = 0;
              if (rTCMTransmitLoopBuffer[i].typeOfRTCMMsg == 1005){
                if (RTCMTransmit1005Timer.getStatus(millis()) >= 0){
                  RTCMTransmit1005Timer.resetTimingNow (millis());
                  if (rTCMviaMQTTisActive){
                    check = mqttPubSubClient.publish ((char*)MQTTMsgTopic.c_str(), pbMQTTMsg, rTCMTransmitLoopBuffer[i].msgLength, true);
                  }
                  else {
                    check = true; //to simulate transmitt if switched off by remote
                  }                
                } 
                else{ // even if not transmitted via MQTT (to increase invervall of non important msg) - free buffer to receive next 1005 msg
                  check = true;
                }
              }
              else { //all other msgs
                if (rTCMviaMQTTisActive){
                  check = mqttPubSubClient.publish ((char*)MQTTMsgTopic.c_str(), pbMQTTMsg, rTCMTransmitLoopBuffer[i].msgLength, MQTT_MSG_RETAINED);
                }
                else {
                  check = true; //to simulate transmitt if switched off by remote
                }   
              }
              #ifdef DEBUG_UART_ENABLED 
                //SerialDebug.print (" Check ");
                //SerialDebug.print (check);
                //SerialDebug.println (" MQTT RTCM Pub ... ");
              #endif 
              if (check){ //msg sent correctly
                rTCMTransmitLoopBuffer[i].alreadySent = true; // stop blocking variable for new input
                rTCMTransmitLoopBuffer[i].sending = false; // stop blocking variable for new input
                MQTTWaitBetweenSendingMsg.resetTimingNow (millisOfOutput);
                #ifdef DEBUG_UART_ENABLED 
                  //SerialDebug.print (" Check ");
                  SerialDebug.print (check);
                  SerialDebug.println (" MQTT Pub OK ");
                #endif 
              } 
              else { //msg NOT sent correctly
                rTCMTransmitLoopBuffer[i].sending = false; // stop blocking variable to give a nother try to send
                rTCMTransmitLoopBuffer[i].readyToSend = true; // stop blocking variable to give a nother try to send
                MQTTWaitBetweenSendingMsg.resetTimingNow (millisOfOutput);
                #ifdef DEBUG_UART_ENABLED 
                  //SerialDebug.print (" Check ");
                  SerialDebug.print (check);
                  SerialDebug.println (" MQTT Pub NOT OK, try again ");
                #endif 
                if (millisOfOutput - rTCMTransmitLoopBuffer[i].millisTimeOfReceive > MQTT_MAX_TIME_FOR_RESEND_MSG){ // to wait longer is not usefull, next msg need the space
                  rTCMTransmitLoopBuffer[i].alreadySent = true; // stop blocking variable for new input
                  rTCMTransmitLoopBuffer[i].sending = false; // stop blocking variable for new input
                  rTCMTransmitLoopBuffer[i].readyToSend = false; // stop blocking variable for new input
                }
              }
              goto end_check_all_transmitted_for_loop; //break; is not working correct //to start new timing for next transmit
            } 
            else { //Nothing to send from buffer, epoch completed
              #ifdef DEBUG_UART_ENABLED 
                //SerialDebug.print ("Nothing to send from buffer, epoch completed ");
                //SerialDebug.println (rTCMTransmitLoopBufferTXEpoch);
              #endif
            }
          }
        end_check_all_transmitted_for_loop:
        int not_needed_just_for_goto = 0;
        not_needed_just_for_goto++;
        #ifdef DEBUG_UART_ENABLED 
          //SerialDebug.print (" EXIT_TX ");
        #endif
        }
        else {// wait between sending of MQTT Msg to avoid overflow/losses
          #ifdef DEBUG_UART_ENABLED 
            SerialDebug.println ("Delay - Waiting to transmitt next MQTTMsg ");;
          #endif
        }
      #endif  
    }
  }
}