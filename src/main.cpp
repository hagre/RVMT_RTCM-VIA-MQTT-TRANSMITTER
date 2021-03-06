/*
INFO: This Software is made for my private use. I like to share, but for further details read the LICENCE file!

RVMT (RTCM VIA MQTT TRANSMITTER) is using the MQTT protocol (as a secure and opensource alternative to NTRIP) to get RTK correction data for my rover GPS units.

ESP32 firmware
Basic selection via #define in the top of this code

by hagre 
06 2019 - 2020 
*/
#define VERSION 2
#define SUB_VERSION 5

#define PROT_VERSION 0
#define PROT_SUB_VERSION 3

//Features
#define USE_MY_SECRETS //self explaining (more or less just for me to use)
#define DEBUG_UART_ENABLED //enable or disable with // in front of #define     //self explaining (more or less just for me to use)
#define WIFI_CONNECTED_NODE 
//or
//#define ETHERNET_CONNECTED_NODE //enable or disable with // in front of #define
#define MQTT_VIA_SECURE_WIFI_NODE
//or
//#define MQTT_VIA_NOT_SECURE_WIFI_NODE
#define MQTT_BROKER_VIA_HOSTNAME
//or
//#define MQTT_BROKER_VIA_IP  
#define RTCM_VIA_MQTT_TRANSMITTING_WITH_PASSWORD

//Settings
#ifdef USE_MY_SECRETS
  #include "secrets/secrets_server.h"
#endif
#ifndef LAN_IP_RANGE
  #define LAN_IP_RANGE 20 //43 or 20 config as required
#endif  
#define YOUR_WIFI_HOSTNAME "RTCM_MQTT_SENDER"
#define IP_1_THIS_NODE 192
#define IP_2_THIS_NODE 168
#define IP_3_THIS_NODE LAN_IP_RANGE
#define IP_4_THIS_NODE 115

#ifdef WIFI_CONNECTED_NODE 
  #ifndef YOUR_WIFI_SSID
    #define YOUR_WIFI_SSID "WLAN"
  #endif
  #ifndef YOUR_WIFI_PASSWORD
    #define YOUR_WIFI_PASSWORD "Password"
  #endif
  //#define WIFI_WAIT_FOR_CONNECTION 10000 //ms time - change default 10000 ms if required
  //#define WIFI_WAIT_FOR_RECONNECTION 5000 //ms time - change default 5000 ms if required
  #ifndef YOUR_WIFI_HOSTNAME
    #define YOUR_WIFI_HOSTNAME "RTCM_MQTT_NOTE"
  #endif 
#endif 

#ifdef RTCM_VIA_MQTT_TRANSMITTING_WITH_PASSWORD
  #ifndef MQTT_BROKER_USERNAME
    #define MQTT_BROKER_USERNAME "MQTTuser" //config as required 
  #endif
  #ifndef MQTT_BROKER_PASSWORD
    #define MQTT_BROKER_PASSWORD "MQTTpassword" //config as required 
  #endif
#endif

#ifndef MQTT_CLIENT_ID_FOR_BROKER
  #define MQTT_CLIENT_ID_FOR_BROKER "TestClient" //config as required 
#endif

#define MQTT_VERSION 4 //4 = MQTT_VERSION_3_1 // 3 = MQTT_VERSION_3_1_1 

#ifdef MQTT_BROKER_VIA_IP
  #ifndef MQTT_BROKER_1_IP 
    #define MQTT_BROKER_1_IP 10 //config as required //for unsecure LAN connection
  #endif
  #ifndef MQTT_BROKER_2_IP 
    #define MQTT_BROKER_2_IP 0 //config as required //for unsecure LAN connection
  #endif
  #ifndef MQTT_BROKER_3_IP 
    #define MQTT_BROKER_3_IP 0 //config as required //for unsecure LAN connection
  #endif
  #ifndef MQTT_BROKER_4_IP 
    #define MQTT_BROKER_4_IP 1 //config as required //for unsecure LAN connection
  #endif
#endif  
#ifdef MQTT_BROKER_VIA_HOSTNAME
  #ifndef HOSTNAME_OF_MQTT_BROKER
    #define HOSTNAME_OF_MQTT_BROKER "test.mosquitto.com" //config as required  //for secure INTERNET connection
  #endif
#endif

#ifdef MQTT_VIA_SECURE_WIFI_NODE
  #define MQTT_CONNECTION_PORT 8883 // 8883 TLS //config as required
#endif  
#ifdef MQTT_VIA_NOT_SECURE_WIFI_NODE
  #define MQTT_CONNECTION_PORT 8882 //8882 = not secure on LAN only //config as required
#endif

#define MQTT_SET_KEEPALIVE 30 //15 = 15sec
#define MQTT_SET_SOCKET_TIMEOUT 40 //20sec

#define MQTT_WAIT_FOR_SERVER_CONNECTION 10000 //ms time - default is 30000 ms, change only if required
#define MQTT_WAIT_FOR_SERVER_RECONNECTION 5000 //ms time - default is 15000 ms, change only if required
#define MQTT_WAIT_BETWEEN_SENDING_MSG 50 // 30ms //config as required
#define MQTT_MAX_TIME_FOR_RESEND_MSG 1000 // 10000ms //config as required
#define MQTT_MSG_RETAINED false // false //config as required

#define MQTT_RTCM_TOPIC_INIT "RTK/Base/"

#ifndef MQTT_RTCM_BASE_NAME
  #define MQTT_RTCM_BASE_NAME "TEST01" //config as required 
#endif

#define MQTT_LASTWILL_QOS 0
#define MQTT_LASTWILL_RETAIN 0
#define MQTT_LASTWILL_MSG "OFFLINE"

#define MQTT_BUFFER_SIZE 1024
#define MQTT_MAX_PACKET_SIZE MQTT_BUFFER_SIZE

#define RTCM_VIA_MQTT_UART_TRANSMITTING_BOUD 230400 //config as required 
#define RTCM_UART_HARDWARE_PORT 2 //config as required //set 2, USB == 0, 1 == UART1 F9P, 2 == UART2 F9P rewire rquired not fitting on ardusimple
#define RTCM_BUFFER_SIZE 1024 //config as required
#define RTCM_LOOP_BUFFER_SIZE 10 //10 config as required // •  RTCM 1005 Stationary RTK reference station ARP•  RTCM 1074 and 1077 GPS •  RTCM 1084 and 1087 GLONASS •  RTCM 1094 ans 1097 Galileo •  RTCM 1124 and 1127 BeiDou•  RTCM 1230 GLONASS code-phase biases

#define TOPIC_MSG_BUFFER_LENGTH 100
#define CONTENT_MSG_BUFFER_LENGTH 100
#define NR_OF_PROTOCOL_MSG_BUFFER 20

#define RTCM_MSG_CHECK_IN_USE_INTERVAL 1000 //
#define RTCM_MAX_IN_USE_DELAY 65000
#define RTCM_IN_USE_MSG_UPDATE_DELAY 1000 //10000 
#define LAST_RTCM_INPUT_WAITING_TIME 10000 //10000

#ifdef DEBUG_UART_ENABLED 
  #define DEBUG_UART_HARDWARE_PORT 0 //config as required // USB == 0
  #define DEBUG_UART_BOUD 115200 //config as required 
  #define DEBUG_WAITING_TIME 1000 //config as required 
#endif
// -------------------------------- End of Config -----------------------------------------------

// -------------------------------- Begin of Program --------------------------------------------
//sanitycheck
#ifdef WIFI_CONNECTED_NODE
  #ifdef ETHERNET_CONNECTED_NODE
    #error "ERROR: select one: WIFI or ETHERNET"
  #endif
#endif

//include libraries
#include <Arduino.h>
#include "verysimpletimer.h"
VerySimpleTimer MQTTWaitBetweenSendingMsg;
VerySimpleTimer RTCMMsgCheckInUseTimer;

#include <rtcmstreamsplitter.h>
RTCMStreamSplitter RTCMStream; 


#define UART0RX 3 //ESP32 GPIOs
#define UART0TX 1
#define UART1RX 16 //26 //ESP32 look like UNO - modified for Ardusimpleconnection 
#define UART1TX 17 //12 problem SPI internal //ESP32 look like UNO - modified for Ardusimpleconnection 
#define UART2RX 13 //16 //ESP32 look like UNO - modified for Ardusimpleconnection 
#define UART2TX 26 //17 //ESP32 look like UNO - modified for Ardusimpleconnection 

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

#ifdef DEBUG_UART_ENABLED
  uint16_t debug_count = 0;
  VerySimpleTimer DebugLoopTimer;
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

struct RTCMTransmitBuffer_t {
  bool readyToSend = false;
  bool alreadySent = true; //clear to input first msg, to start sending
  unsigned long nrOfInternalEpoche;
  unsigned long millisTimeOfReceiveForSending;
  unsigned long millisTimeOfReceiveForCalculation;
  unsigned long lastRTCMinUseUpdateTime;
  int16_t typeOfRTCMMsg = 0;
  byte RTCMMsg [RTCM_BUFFER_SIZE]; 
  uint16_t msgLength = 0;
  bool isUsedTypeOfMsg = false;
  int16_t averageReceiveIntervalCalculated = 0;
  int16_t averageReceiveIntervalSent = 0; //-1 ?
} rTCMTransmitLoopBuffer [RTCM_LOOP_BUFFER_SIZE];

struct RoofNodeTransmitBuffer_t {
  bool readyToSend = false;
  bool alreadySent = true; //clear to input first msg, to start sending
  int16_t typeOfRTCMMsg = 0;
  char topicMsg [TOPIC_MSG_BUFFER_LENGTH];
  uint16_t topicLength = 0;
  char contentMsg [CONTENT_MSG_BUFFER_LENGTH]; 
  uint16_t msgLength = 0;
  uint8_t qOS = 0;
  bool retain = false;
} roofNodeTransmitBuffer [NR_OF_PROTOCOL_MSG_BUFFER];

uint8_t serialRTCMtoMQTTBuffer[MQTT_BUFFER_SIZE];

unsigned long rTCMTransmitLoopBufferRXEpoch = 0;
unsigned long rTCMTransmitLoopBufferTXEpoch = 0;

unsigned long lastRTCMinputTime = 0;
unsigned long lastRTCMinputWaitingTime = LAST_RTCM_INPUT_WAITING_TIME;

const IPAddress Node_IP (IP_1_THIS_NODE, IP_2_THIS_NODE, IP_3_THIS_NODE, IP_4_THIS_NODE);

bool rTCMviaMQTTisActive = true;
uint8_t statusBaseStation = 0;  //0 == OFFLINE, 1 = ERROR, 2 = STBY, 3 = ONLINE

int8_t LANStatus = -5; //Connected to Network //WIFI or ETHERNET //-5 = init, -2 = just disconnected, -1 = wait to reconnect, 0 = disconnected, 1 = connecting, 2 = just connected,  3 = still connected
int8_t MQTTStatus = -5; //Connected to MQTT broker // -5 init, -3 LAN just disconnected, -2 just disconnected, -1 wait to reconnect, 0 disconnected, 1 connecting, 2 just connected, 3 subscribing, 4 subscribed and connected

String lastWillTopic;

#ifdef MQTT_BROKER_VIA_HOSTNAME
  const char* mQTTBrokerHostName = HOSTNAME_OF_MQTT_BROKER;
#endif

#ifdef MQTT_BROKER_VIA_IP
  const IPAddress mQTTBrokerIP (MQTT_BROKER_1_IP, MQTT_BROKER_2_IP, MQTT_BROKER_3_IP, MQTT_BROKER_4_IP); 
#endif  

#ifdef RTCM_VIA_MQTT_TRANSMITTING_WITH_PASSWORD
  const char * mqttUsername = MQTT_BROKER_USERNAME;
  const char * mqttPassword = MQTT_BROKER_PASSWORD;
#endif

#ifdef WIFI_CONNECTED_NODE
  #include <WiFi.h>
  #include <SyncWifiConnectionESP32.h>
  SyncWifiConnectionESP32 SyncWifiConnection;
  const char* ssid     = YOUR_WIFI_SSID; 
  const char* password = YOUR_WIFI_PASSWORD;

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

  #include <SyncMQTTConnectionESP32.h>
  SyncMQTTConnectionESP32 syncMQTTConnection; 
  const char* mqttPubSubClientId = MQTT_CLIENT_ID_FOR_BROKER;
#endif

#ifdef ETHERNET_CONNECTED_NODE
  // LAN ToDo... ------------------------------------------------------------------------------------------------ToDO-----------------------------------------------
#endif

   
bool ReadRTCMSerialToBuffer (){
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
    #ifdef DEBUG_UART_ENABLED 
      //SerialDebug.print (".");
      //SerialDebug.print (char(newMsgChar));
    #endif

    typeOfMsg = RTCMStream.inputByte (newMsgChar);
    if (typeOfMsg > 0) { // new msg completly arrived
      #ifdef DEBUG_UART_ENABLED 
        SerialDebug.print (" Type of RTCM ");
        SerialDebug.println (typeOfMsg);
      #endif

      unsigned long millisOfInput = millis();
      lastRTCMinputTime = millisOfInput;

      for (int x = 0; x < RTCM_LOOP_BUFFER_SIZE; x++){
        if (typeOfMsg == rTCMTransmitLoopBuffer[x].typeOfRTCMMsg){
            #ifdef DEBUG_UART_ENABLED 
              SerialDebug.print (" this nbr is ");
              SerialDebug.println (x);
            #endif

          //for average calculation (MNTP list)
          if (rTCMTransmitLoopBuffer[x].isUsedTypeOfMsg){ //was already in use
            #ifdef DEBUG_UART_ENABLED 
              SerialDebug.print (" rTCMTransmitLoopBuffer is Used ");
            #endif
            float calculation = (millisOfInput - rTCMTransmitLoopBuffer[x].millisTimeOfReceiveForCalculation) / 1000.0;
            rTCMTransmitLoopBuffer[x].averageReceiveIntervalCalculated = round (calculation);
            #ifdef DEBUG_UART_ENABLED
              //SerialDebug.print (calculation);
              //SerialDebug.print (" Calculated Interval ");
              //SerialDebug.println (rTCMTransmitLoopBuffer[x].averageReceiveIntervalCalculated);
            #endif
          }
          else { //was not in use, but is now
            rTCMTransmitLoopBuffer[x].isUsedTypeOfMsg = true;
            #ifdef DEBUG_UART_ENABLED 
              SerialDebug.print (" Call rTCMTransmitLoopBuffer used now ");
            #endif
          }
          rTCMTransmitLoopBuffer[x].millisTimeOfReceiveForCalculation = millisOfInput;

          if (rTCMTransmitLoopBufferRXEpoch - rTCMTransmitLoopBuffer[x].nrOfInternalEpoche > 0){
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
              //SerialDebug.println (millisOfInput - rTCMTransmitLoopBuffer[i].millisTimeOfReceiveForSending);
            #endif

            rTCMTransmitLoopBuffer[x].millisTimeOfReceiveForSending = millisOfInput;
            rTCMTransmitLoopBuffer[x].typeOfRTCMMsg = typeOfMsg;
            for (uint16_t z = 0; z < RTCMStream.outputStreamLength; z++){
              rTCMTransmitLoopBuffer[x].RTCMMsg[z] = RTCMStream.outputStream[z];
              #ifdef DEBUG_UART_ENABLED 
                //SerialDebug.print (rTCMTransmitLoopBuffer[i].RTCMMsg, HEX); 
              #endif
            }
            rTCMTransmitLoopBuffer[x].msgLength = RTCMStream.outputStreamLength;
            rTCMTransmitLoopBuffer[x].nrOfInternalEpoche = rTCMTransmitLoopBufferRXEpoch;
            rTCMTransmitLoopBuffer[x].alreadySent = false;
            rTCMTransmitLoopBuffer[x].readyToSend = true; 
          }
          else { //This Msg buffer still not complete transmitted
            #ifdef DEBUG_UART_ENABLED 
              SerialDebug.println (" RTCM Msg Buffer blocked - sending not marked as finished");
            #endif
          }
        }
      }
    #ifdef DEBUG_UART_ENABLED
      //SerialDebug.println (" Break after recognized Msg ");
    #endif

    return true;
    }
  }
  
  return false;
}

bool MQTTTransmitMsg (int8_t actualMQTTStatus){
  if (actualMQTTStatus == 4) {
    unsigned long millisOfOutput = millis();

    #ifdef DEBUG_UART_ENABLED 
      //SerialDebug.println (MQTTWaitBetweenSendingMsg.getStatus (millisOfOutput,false));
    #endif

    if (MQTTWaitBetweenSendingMsg.getStatus (millisOfOutput, false) >= 0){ // wait between sending of MQTT Msg to avoid overflow/losses
      #ifdef DEBUG_UART_ENABLED 
        //SerialDebug.println (" Ready to send, ");
      #endif

      for (int i = 0; i < RTCM_LOOP_BUFFER_SIZE; i++){// check complete buffer (rTCMTransmitLoopBuffer)
        #ifdef DEBUG_UART_ENABLED 
          //SerialDebug.print (rTCMTransmitLoopBuffer[i].readyToSend);
        #endif

        if (rTCMTransmitLoopBuffer[i].readyToSend){
          #ifdef DEBUG_UART_ENABLED 
                SerialDebug.print ("Ready to send next MQTT found  ");
          #endif
          if (rTCMTransmitLoopBuffer[i].nrOfInternalEpoche - rTCMTransmitLoopBufferTXEpoch > 0) {
            #ifdef DEBUG_UART_ENABLED 
              SerialDebug.print (" Transmitting new epoch! ");
              if (rTCMTransmitLoopBuffer[i].nrOfInternalEpoche - rTCMTransmitLoopBufferTXEpoch > 1){
                SerialDebug.print (" Missed epoch! ");
                SerialDebug.println (rTCMTransmitLoopBuffer[i].nrOfInternalEpoche - rTCMTransmitLoopBufferTXEpoch);
              }
            #endif
            rTCMTransmitLoopBufferTXEpoch = rTCMTransmitLoopBuffer[i].nrOfInternalEpoche;
          }
          // preperation to transmitt 
          String MQTTMsgType = String (rTCMTransmitLoopBuffer[i].typeOfRTCMMsg);
          String MQTTMsgTopic = "";
          MQTTMsgTopic = MQTT_RTCM_TOPIC_INIT;
          MQTTMsgTopic = MQTTMsgTopic + MQTT_RTCM_BASE_NAME;
          MQTTMsgTopic = MQTTMsgTopic + "/" ;
          MQTTMsgTopic = MQTTMsgTopic + MQTTMsgType;

          char bMQTTMsg [RTCM_BUFFER_SIZE];
          //char* pbMQTTMsg = bMQTTMsg;
          #ifdef DEBUG_UART_ENABLED 
            SerialDebug.print (" msgLength ");
            SerialDebug.println (rTCMTransmitLoopBuffer[i].msgLength);
          #endif

          for (uint16_t x = 0; x < rTCMTransmitLoopBuffer[i].msgLength; x++){
            bMQTTMsg [x]= rTCMTransmitLoopBuffer[i].RTCMMsg[x];
            #ifdef DEBUG_UART_ENABLED 
              //SerialDebug.print (bMQTTMsg[x], HEX); 
            #endif
          }
          #ifdef DEBUG_UART_ENABLED 
            //SerialDebug.println ("");
            //SerialDebug.println (bMQTTMsg); 
          #endif

          int16_t check = 0;

          if (rTCMviaMQTTisActive){
            check = syncMQTTConnection.publish (MQTTMsgTopic.c_str(), (uint8_t *) bMQTTMsg, rTCMTransmitLoopBuffer[i].msgLength, MQTT_MSG_RETAINED);
          }
          else {
            check = true; //to simulate transmitt if switched off by remote
          }   
          
          #ifdef DEBUG_UART_ENABLED 
            SerialDebug.print (" Check ");
            SerialDebug.print (check);
            SerialDebug.println (" MQTT RTCM Pub ... ");
          #endif 
          if (check){ //msg sent correctly
            rTCMTransmitLoopBuffer[i].alreadySent = true; // stop blocking variable for new input
            rTCMTransmitLoopBuffer[i].readyToSend = false; // stop blocking variable for new input
            MQTTWaitBetweenSendingMsg.resetTimingNow (millisOfOutput);
            #ifdef DEBUG_UART_ENABLED 
              SerialDebug.print (" Check ");
              SerialDebug.print (check);
              SerialDebug.println (" MQTT Pub OK ");
            #endif 
            return true;  //to start new timing for next transmit
          } 
          else { //msg NOT sent correctly
            MQTTWaitBetweenSendingMsg.resetTimingNow (millisOfOutput);
            #ifdef DEBUG_UART_ENABLED 
              //SerialDebug.print (" Check ");
              SerialDebug.print (check);
              SerialDebug.println (" MQTT Pub NOT OK, try again ");
            #endif 
            if (millisOfOutput - rTCMTransmitLoopBuffer[i].millisTimeOfReceiveForSending > MQTT_MAX_TIME_FOR_RESEND_MSG){ // to wait longer is not usefull, next msg need the space
              rTCMTransmitLoopBuffer[i].alreadySent = true; // stop blocking variable for new input
              rTCMTransmitLoopBuffer[i].readyToSend = false; // stop blocking variable for new input
            }
            return false;
          }
        } 
        else { //Nothing to send from buffer, epoch completed
          #ifdef DEBUG_UART_ENABLED 
            //SerialDebug.print ("Nothing to send from buffer, epoch completed ");
            //SerialDebug.println (rTCMTransmitLoopBufferTXEpoch);
          #endif
        }
      }

      for (int i = 0; i < NR_OF_PROTOCOL_MSG_BUFFER; i++){// check complete buffer (roofNodeTransmitBuffer) ---------------------------------------------------------------------------------------------------------------------------------------------------------------
        #ifdef DEBUG_UART_ENABLED 
          //SerialDebug.print (roofNodeTransmitBuffer[i].readyToSend);
          //SerialDebug.print (" TransmittBufferSend, ");
          //SerialDebug.println (i);
        #endif

        if (roofNodeTransmitBuffer[i].readyToSend){
          #ifdef DEBUG_UART_ENABLED 
            SerialDebug.print ("Ready to send next Protocol MQTT found  ");
          #endif

          //char* pbMQTTMsg =  roofNodeTransmitBuffer[i].contentMsg;

          String MQTTMsgTopic = "";
          for (uint16_t x = 0; x < roofNodeTransmitBuffer[i].topicLength; x++){
            MQTTMsgTopic = MQTTMsgTopic + char(roofNodeTransmitBuffer[i].topicMsg[x]);
          }

          if (MQTTMsgTopic == ""){
            MQTTMsgTopic = "Test";
          }

          #ifdef DEBUG_UART_ENABLED 
            //SerialDebug.println (MQTTMsgTopic);
            //SerialDebug.println (roofNodeTransmitBuffer[i].contentMsg);
          #endif 

          int16_t check = 0;
          check = syncMQTTConnection.publish (MQTTMsgTopic.c_str(), (uint8_t *) roofNodeTransmitBuffer[i].contentMsg, roofNodeTransmitBuffer[i].msgLength, roofNodeTransmitBuffer[i].retain);
          //not supported by PubSubClient Library
          //check = syncMQTTConnection.publish ((char*)MQTTMsgTopic.c_str(), pbMQTTMsg, roofNodeTransmitBuffer[i].msgLength, roofNodeTransmitBuffer[i].qOS, roofNodeTransmitBuffer[i].retain);
          #ifdef DEBUG_UART_ENABLED 
            SerialDebug.print (" Check ");
            SerialDebug.print (check);
            SerialDebug.println (" MQTT RTCM Pub ... ");
          #endif 
          if (check){ //msg sent correctly
            roofNodeTransmitBuffer[i].alreadySent = true; // stop blocking variable for new input
            roofNodeTransmitBuffer[i].readyToSend = false; // stop blocking variable for new input
            MQTTWaitBetweenSendingMsg.resetTimingNow (millisOfOutput);
            #ifdef DEBUG_UART_ENABLED 
              //SerialDebug.print (" Check ");
              SerialDebug.print (check);
              SerialDebug.println (" MQTT Pub OK ");
            #endif 
            return true;  //to start new timing for next transmit
          } 
          else { //msg NOT sent correctly
            MQTTWaitBetweenSendingMsg.resetTimingNow (millisOfOutput);
            #ifdef DEBUG_UART_ENABLED 
              //SerialDebug.print (" Check ");
              SerialDebug.print (check);
              SerialDebug.println (" MQTT Pub NOT OK, try again ");
            #endif 
            return false;
          }
        } 
        else { //Nothing to send from buffer, epoch completed
          #ifdef DEBUG_UART_ENABLED 
            //SerialDebug.println ("Nothing to send from buffer, check completed ");
          #endif
        }
      } // end of for loop
    } // END of wait between sending of MQTT Msg to avoid overflow/losses
    else {
      #ifdef DEBUG_UART_ENABLED 
        //SerialDebug.println ("Delay - Waiting to transmitt next MQTTMsg ");;
      #endif
    }
  }
  return false; // MQTT and LAN not connected
}


void SetRoofNodeTransmitBufferStr (uint16_t nbr, String topic, String msg, int16_t typeOfRTCMMsg, uint8_t qOS, bool retain){
  for (uint16_t i = 0; i < topic.length(); i++){
    roofNodeTransmitBuffer[nbr].topicMsg[i] = topic[i];
  }
  for (uint16_t i = 0; i < msg.length(); i++){
    roofNodeTransmitBuffer[nbr].contentMsg[i] = msg[i];
  }
  roofNodeTransmitBuffer[nbr].topicLength = topic.length();
  roofNodeTransmitBuffer[nbr].msgLength = msg.length();
  roofNodeTransmitBuffer[nbr].qOS = qOS;
  roofNodeTransmitBuffer[nbr].typeOfRTCMMsg = typeOfRTCMMsg;
  roofNodeTransmitBuffer[nbr].retain = retain;
  roofNodeTransmitBuffer[nbr].alreadySent = false;
  roofNodeTransmitBuffer[nbr].readyToSend = true;
}

void SetRoofNodeTransmitBufferInt (uint16_t nbr, String topic, int16_t msg, int16_t typeOfRTCMMsg, uint8_t qOS, bool retain){
  for (uint16_t i = 0; i < topic.length(); i++){
    roofNodeTransmitBuffer[nbr].topicMsg[i] = topic[i];
  }
  char transformMsg [5];
  snprintf(transformMsg, sizeof(transformMsg), "%d", msg);
  String transformMsgString (transformMsg);

  for (uint16_t i = 0; i < transformMsgString.length(); i++){
    roofNodeTransmitBuffer[nbr].contentMsg[i] = transformMsgString[i];
  }

  roofNodeTransmitBuffer[nbr].topicLength = topic.length();
  roofNodeTransmitBuffer[nbr].msgLength = transformMsgString.length();
  roofNodeTransmitBuffer[nbr].qOS = qOS;
  roofNodeTransmitBuffer[nbr].typeOfRTCMMsg = typeOfRTCMMsg;
  roofNodeTransmitBuffer[nbr].retain = retain;
  roofNodeTransmitBuffer[nbr].alreadySent = false;
  roofNodeTransmitBuffer[nbr].readyToSend = true;
}

void ChangeIntMsgInRoofNodeTransmitBuffer (uint16_t nbr, int16_t msg, bool retain){
  char transformMsg [5];
  snprintf(transformMsg, sizeof(transformMsg), "%d", msg);
  String transformMsgString (transformMsg);

  for (uint16_t i = 0; i < transformMsgString.length(); i++){
    roofNodeTransmitBuffer[nbr].contentMsg[i] = transformMsgString[i];
  }
  roofNodeTransmitBuffer[nbr].retain = retain;
  roofNodeTransmitBuffer[nbr].msgLength = transformMsgString.length();
  roofNodeTransmitBuffer[nbr].alreadySent = false;
  roofNodeTransmitBuffer[nbr].readyToSend = true;
}

void ChangeStringMsgInRoofNodeTransmitBuffer (uint16_t nbr, String msg, bool retain){

  for (uint16_t i = 0; i < msg.length(); i++){
    roofNodeTransmitBuffer[nbr].contentMsg[i] = msg[i];
  }
  roofNodeTransmitBuffer[nbr].msgLength = msg.length();
  roofNodeTransmitBuffer[nbr].retain = retain;
  roofNodeTransmitBuffer[nbr].alreadySent = false;
  roofNodeTransmitBuffer[nbr].readyToSend = true;
}

void receivedMQTTCallback(char* topic, byte* payload, unsigned int length) {
 //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++check function+++++++++++++++++++++++++++++++++++++++++++
  #ifdef DEBUG_UART_ENABLED 
    SerialDebug.println (" Callback ");
  #endif

  if (strcmp(topic, syncMQTTConnection.getTopicFromBuffer(2)) == 0){ //RTCM Transmitter ON / OFF
    #ifdef DEBUG_UART_ENABLED 
      SerialDebug.println (syncMQTTConnection.getTopicFromBuffer(2));
    #endif 

    if (((char)payload[0]) == 'O'){ //ON
      if (((char)payload[1]) == 'N'){
        rTCMviaMQTTisActive = true;
        #ifdef DEBUG_UART_ENABLED
          SerialDebug.println("Message received: ON NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN");
        #endif
      }
    }
    if (((char)payload[0]) == 'O'){ //OFF
      if (((char)payload[1]) == 'F'){
        //if (((char)payload[2]) == 'F'){
          rTCMviaMQTTisActive = false;
          #ifdef DEBUG_UART_ENABLED
            SerialDebug.println("Message received: OFF FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF ");
          #endif
        //}
      }
    }
  }
  
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

void initRoofNodeTransmitBuffer (String topicFirstPart){

  String tempTopic, tempValueStr;
  uint16_t tempValueInt;

  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/Status/Operation";
  tempValueStr = "STBY";
  SetRoofNodeTransmitBufferStr (0, tempTopic, tempValueStr, 0, 1, true);
  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/Status/FirmwareVer";
  tempValueStr = "";
  tempValueStr = tempValueStr + VERSION;
  tempValueStr = tempValueStr + ".";
  tempValueStr = tempValueStr + SUB_VERSION;
  SetRoofNodeTransmitBufferStr (1, tempTopic, tempValueStr, 0, 1, true);
  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/Status/ProtocolVer";
  tempValueStr = "";
  tempValueStr = tempValueStr + PROT_VERSION;
  tempValueStr = tempValueStr + ".";
  tempValueStr = tempValueStr + PROT_SUB_VERSION;
  SetRoofNodeTransmitBufferStr (2, tempTopic, tempValueStr, 0, 1, true);
  
  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/Status/Error";
  tempValueStr = "Booting";
  SetRoofNodeTransmitBufferStr (3, tempTopic, tempValueStr, 0, 0, false);
  
  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/1005";
  tempValueInt = 0;
  SetRoofNodeTransmitBufferInt (4, tempTopic, tempValueInt, 1005, 0, false);
  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/1074";
  tempValueInt = 0;
  SetRoofNodeTransmitBufferInt (5, tempTopic, tempValueInt, 1074, 0, false);
  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/1077";
  tempValueInt = 0;
  SetRoofNodeTransmitBufferInt (6, tempTopic, tempValueInt, 1077, 0, false);
  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/1084";
  tempValueInt = 0;
  SetRoofNodeTransmitBufferInt (7, tempTopic, tempValueInt, 1084, 0, false);
  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/1087";
  tempValueInt = 0;
  SetRoofNodeTransmitBufferInt (8, tempTopic, tempValueInt, 1087, 0, false);
  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/1094";
  tempValueInt = 0;
  SetRoofNodeTransmitBufferInt (9, tempTopic, tempValueInt, 1094, 0, false);
  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/1097";
  tempValueInt = 0;
  SetRoofNodeTransmitBufferInt (10, tempTopic, tempValueInt, 1097, 0, false);
  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/1124";
  tempValueInt = 0;
  SetRoofNodeTransmitBufferInt (11, tempTopic, tempValueInt, 1124, 0, false);
  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/1127";
  tempValueInt = 0;
  SetRoofNodeTransmitBufferInt (12, tempTopic, tempValueInt, 1127, 0, false);
  tempTopic = topicFirstPart;
  tempTopic = tempTopic + MQTT_RTCM_BASE_NAME;
  tempTopic = tempTopic + "/1230";
  tempValueInt = 0;
  SetRoofNodeTransmitBufferInt (13, tempTopic, tempValueInt, 1230, 0, false);
}

void setup() { // -------------------------------- S E T U P --------------------------------------------

  #ifdef DEBUG_UART_ENABLED
    SerialDebug.begin (serialDebugPortBoud, SERIAL_8N1, DEBUG_UART_RX, DEBUG_UART_TX); //Debug output, usually USB Port
    SerialDebug.println ("Setup");
    DebugLoopTimer.setIntervalMs (DEBUG_WAITING_TIME);
  #endif
  
  #ifdef WIFI_CONNECTED_NODE 
    #ifdef DEBUG_UART_ENABLED
      SerialDebug.println ("Starting WIFI");
      SyncWifiConnection.setWifiDebugSerial (&SerialDebug);
    #endif
    SyncWifiConnection.init (WIFI_STA, Node_IP, YOUR_WIFI_HOSTNAME, YOUR_WIFI_SSID, YOUR_WIFI_PASSWORD); 
  #endif 

  #ifdef ETHERNET_CONNECTED_NODE
    #ifdef DEBUG_UART_ENABLED
      SerialDebug.println ("Starting ETERNET modul");
    #endif 
    //init ETHERNET Modul and connection
    //todo ...
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

  lastWillTopic = MQTT_RTCM_TOPIC_INIT;
  lastWillTopic = lastWillTopic + MQTT_RTCM_BASE_NAME;
  lastWillTopic = lastWillTopic + "/Status/Operation";

  #ifdef DEBUG_UART_ENABLED
    SerialDebug.println ("Starting MQTT");
    syncMQTTConnection.setMQTTDebugSerial (&SerialDebug);
  #endif

  #ifdef MQTT_BROKER_VIA_HOSTNAME
    syncMQTTConnection.setMQTTConnection (mqttPubSubClientId, mqttUsername, mqttPassword, true, mqttLANClient, mQTTBrokerHostName, MQTT_CONNECTION_PORT, MQTT_MAX_PACKET_SIZE, MQTT_SET_KEEPALIVE, MQTT_SET_SOCKET_TIMEOUT);
  #endif

  #ifdef MQTT_BROKER_VIA_IP
      syncMQTTConnection.setMQTTConnection (mqttPubSubClientId, mqttUsername, mqttPassword, true, mqttLANClient, mQTTBrokerIP, MQTT_CONNECTION_PORT, MQTT_MAX_PACKET_SIZE, MQTT_SET_KEEPALIVE, MQTT_SET_SOCKET_TIMEOUT);
  #endif

  syncMQTTConnection.setMQTTLastWill (lastWillTopic.c_str(), MQTT_LASTWILL_QOS, MQTT_LASTWILL_RETAIN, MQTT_LASTWILL_MSG);
  syncMQTTConnection.setMQTTCallback (receivedMQTTCallback);

  String subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
  subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
  subscriptionTopic = subscriptionTopic + "/Command/In/RTCM"; 
  syncMQTTConnection.addSubscriptionToTable (0, subscriptionTopic.c_str(), subscriptionTopic.length ());

  subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
  subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
  subscriptionTopic = subscriptionTopic + "/Command/In/GPS";
  syncMQTTConnection.addSubscriptionToTable (1, subscriptionTopic.c_str(), subscriptionTopic.length ());

  subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
  subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
  subscriptionTopic = subscriptionTopic + "/Command/In/Transmitter";
  syncMQTTConnection.addSubscriptionToTable (2, subscriptionTopic.c_str(), subscriptionTopic.length ());

  subscriptionTopic = "#";
  //syncMQTTConnection.addSubscriptionToTable (3, subscriptionTopic.c_str(), subscriptionTopic.length ());

/*
  subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
  subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
  subscriptionTopic = subscriptionTopic + "/Command/In/Serial";
  syncMQTTConnection.addSubscriptionToTable (3, (byte*)(subscriptionTopic.c_str()), subscriptionTopic.length ());

  subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
  subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
  subscriptionTopic = subscriptionTopic + "/Serial/In";
  syncMQTTConnection.addSubscriptionToTable (4, (byte*)(subscriptionTopic.c_str()), subscriptionTopic.length ());
*/

  MQTTWaitBetweenSendingMsg.setIntervalMs (MQTT_WAIT_BETWEEN_SENDING_MSG);
  MQTTWaitBetweenSendingMsg.resetTimingNow (millis());
  RTCMMsgCheckInUseTimer.setIntervalMs (RTCM_MSG_CHECK_IN_USE_INTERVAL);
  RTCMMsgCheckInUseTimer.resetTimingNow (millis());

  // initiate Loop buffer variable
  rTCMTransmitLoopBuffer[0].typeOfRTCMMsg = 1005; //RTCM 1005 Stationary RTK reference station ARP
  rTCMTransmitLoopBuffer[1].typeOfRTCMMsg = 1074; //RTCM 1074 GPS MSM4
  rTCMTransmitLoopBuffer[2].typeOfRTCMMsg = 1077; //RTCM 1077 GPS MSM7
  rTCMTransmitLoopBuffer[3].typeOfRTCMMsg = 1084; //RTCM 1084 GLONASS MSM4
  rTCMTransmitLoopBuffer[4].typeOfRTCMMsg = 1087; //RTCM 1087 GLONASS MSM7
  rTCMTransmitLoopBuffer[5].typeOfRTCMMsg = 1094; //RTCM 1094 Galileo MSM4
  rTCMTransmitLoopBuffer[6].typeOfRTCMMsg = 1097; //RTCM 1097 Galileo MSM7
  rTCMTransmitLoopBuffer[7].typeOfRTCMMsg = 1124; //RTCM 1124 BeiDou MSM4
  rTCMTransmitLoopBuffer[8].typeOfRTCMMsg = 1127; //RTCM 1127 BeiDou MSM7
  rTCMTransmitLoopBuffer[9].typeOfRTCMMsg = 1230; //RTCM 1230 GLONASS code-phase biases

  initRoofNodeTransmitBuffer (MQTT_RTCM_TOPIC_INIT);
}

void loop() { // -------------------------------- L O O P --------------------------------------------

  //unsigned long loopTime = millis ();

  #ifdef DEBUG_UART_ENABLED
    //SerialDebug.println ("LOOP");
  #endif

  ReadRTCMSerialToBuffer (); 

  #ifdef WIFI_CONNECTED_NODE
    //WIFI CONNECTION MANAGER
    LANStatus = SyncWifiConnection.loop(millis());
  #endif

  #ifdef ETHERNET_CONNECTED_NODE
    //LAN CONNECTION MANAGER
    //init ETHERNET Modul and connection
    //todo ...
    LANStatus = -2;
  #endif


  int8_t MQTTStatus;
  MQTTStatus = syncMQTTConnection.loop (millis(), LANStatus);
 
  MQTTTransmitMsg (MQTTStatus);  

  int newStatusBaseStation = 2; //i can´t be OFFLINE, have to be 1 = ERROR, 2 = STBY, 3 = ONLINE 
  if (rTCMviaMQTTisActive){ //if transmitting is required it must be stby
    newStatusBaseStation = 3;
  }
  if (millis () - lastRTCMinputTime  > lastRTCMinputWaitingTime){ //but, if it is to long with no new RTCM data, it has to be an error
    newStatusBaseStation = 1; //0 == OFFLINE, 1 = ERROR, 2 = STBY, 3 = ONLINE
  }
  if (newStatusBaseStation != statusBaseStation){ //something changed
    statusBaseStation = newStatusBaseStation;
    if (statusBaseStation == 1){
      ChangeStringMsgInRoofNodeTransmitBuffer (0,"ERROR",true);
    }
    else if (statusBaseStation == 2){
      ChangeStringMsgInRoofNodeTransmitBuffer (0,"STBY",true);
    }
    else if (statusBaseStation == 3){
      ChangeStringMsgInRoofNodeTransmitBuffer (0,"ONLINE",true);
    }
  }

  unsigned long RTCMCheckLoopTime = millis ();
  if (RTCMMsgCheckInUseTimer.getStatus (RTCMCheckLoopTime, true) >= 0){ // wait to check Msg in Use 
    #ifdef DEBUG_UART_ENABLED 
      //SerialDebug.println (" Check RTCM Msg in use, ");
    #endif
    
    for (int i = 0; i < RTCM_LOOP_BUFFER_SIZE; i++){// check complete buffer (rTCMTransmitLoopBuffer)

      if (rTCMTransmitLoopBuffer[i].isUsedTypeOfMsg){
        #ifdef DEBUG_UART_ENABLED 
          //SerialDebug.print (" RTCM Msg ");
          //SerialDebug.print (rTCMTransmitLoopBuffer[i].typeOfRTCMMsg);
          //SerialDebug.println (" is in use ");
        #endif
      
        if ((RTCMCheckLoopTime - rTCMTransmitLoopBuffer[i].millisTimeOfReceiveForCalculation > RTCM_MAX_IN_USE_DELAY) || ((rTCMTransmitLoopBuffer[i].averageReceiveIntervalCalculated > 0) && (RTCMCheckLoopTime - rTCMTransmitLoopBuffer[i].millisTimeOfReceiveForCalculation > rTCMTransmitLoopBuffer[i].averageReceiveIntervalCalculated * 3 * 1000))){
          rTCMTransmitLoopBuffer[i].isUsedTypeOfMsg = false;
          rTCMTransmitLoopBuffer[i].averageReceiveIntervalCalculated = 0;
          #ifdef DEBUG_UART_ENABLED 
            SerialDebug.println (RTCMCheckLoopTime);
            SerialDebug.println (rTCMTransmitLoopBuffer[i].millisTimeOfReceiveForCalculation);
            SerialDebug.println (rTCMTransmitLoopBuffer[i].averageReceiveIntervalCalculated);
            SerialDebug.println (" but unused to long, so RTCM Msg is set to -not in use- ");
          #endif
        }
      }
      
      if (RTCMCheckLoopTime - rTCMTransmitLoopBuffer[i].lastRTCMinUseUpdateTime > RTCM_IN_USE_MSG_UPDATE_DELAY){
        #ifdef DEBUG_UART_ENABLED 
          //SerialDebug.println (" RTCM Status Update possible ");
        #endif
        if (rTCMTransmitLoopBuffer[i].averageReceiveIntervalSent != rTCMTransmitLoopBuffer[i].averageReceiveIntervalCalculated){ //different values detected
          #ifdef DEBUG_UART_ENABLED 
            SerialDebug.println (" different values detected ");
          #endif
          rTCMTransmitLoopBuffer[i].averageReceiveIntervalSent = rTCMTransmitLoopBuffer[i].averageReceiveIntervalCalculated;
          
          for (int x = 0; x < NR_OF_PROTOCOL_MSG_BUFFER; x++){// check complete buffer (roofNodeTransmitBuffer) for corresponding type of Msg
            if (roofNodeTransmitBuffer[x].typeOfRTCMMsg == rTCMTransmitLoopBuffer[i].typeOfRTCMMsg){
              if (rTCMTransmitLoopBuffer[i].isUsedTypeOfMsg){
                ChangeIntMsgInRoofNodeTransmitBuffer (x, rTCMTransmitLoopBuffer[i].averageReceiveIntervalCalculated, true);
                rTCMTransmitLoopBuffer[i].lastRTCMinUseUpdateTime = RTCMCheckLoopTime;
                #ifdef DEBUG_UART_ENABLED 
                  SerialDebug.println (" In use ");
                #endif
              }
              else {
                ChangeIntMsgInRoofNodeTransmitBuffer (x, rTCMTransmitLoopBuffer[i].averageReceiveIntervalCalculated, false);
                rTCMTransmitLoopBuffer[i].lastRTCMinUseUpdateTime = RTCMCheckLoopTime;
                #ifdef DEBUG_UART_ENABLED 
                  SerialDebug.println (" Not in use ");
                #endif
              } 
            }
          }
        }
      }
    } 
  }
}