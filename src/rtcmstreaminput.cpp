#include "Arduino.h"
#include "rtcmstreaminput.h"

RTCMStreamInput::RTCMStreamInput()
{
    _inLine = false;
    _receiveStreamLengthCtr = -1;
}

uint16_t RTCMStreamInput::inputByte(uint8_t input) //ToDo Protocoll check
{ 
    _receiveStreamLengthCtr ++;
    _receiveStream[_receiveStreamLengthCtr] = input;
    if (_receiveStreamLengthCtr < 2){
        _inLine = false;
        return 0;
    }
    else {
        if (_receiveStream[_receiveStreamLengthCtr - 2] == 0xD3 && _inLine == false){ // 211 Präamble
            if ((_receiveStream[_receiveStreamLengthCtr - 1] & B11111100) == 0){ //have to be always == 0
                _inLine = true;
                for (uint16_t i = 0; i < MAX_BUFFFERSIZE; i++){ //reset/clear outputstream
                    outputStream [i] = 0;
                }
                outputStreamLength = 0; //reset/clear outputstream
                _receiveStreamLength = ((_receiveStream[_receiveStreamLengthCtr -1] & B00000011) << 8 ) + (_receiveStream[_receiveStreamLengthCtr]);
                _receiveStream[0] = _receiveStream[_receiveStreamLengthCtr - 2];
                _receiveStream[1] = _receiveStream[_receiveStreamLengthCtr - 1];
                _receiveStream[2] = _receiveStream[_receiveStreamLengthCtr];
                _receiveStreamLengthCtr = 2;
            }
        }
        if (_inLine == true && _receiveStreamLengthCtr >= _receiveStreamLength + 5){
            hashData (0, _receiveStreamLength+3);
            if (_inputStreamHash[0] == _receiveStream[_receiveStreamLengthCtr - 2]){
                if (_inputStreamHash[1] == _receiveStream[_receiveStreamLengthCtr - 1]){ 
                    if (_inputStreamHash[2] == _receiveStream[_receiveStreamLengthCtr]){
                        _inLine = false;
                        uint16_t receiveStreamType = (uint16_t)(_receiveStream[3] << 4 ) + (uint16_t)((uint16_t)(_receiveStream[4] & B11110000) >> 4);
                        outputStreamLength = _receiveStreamLength + 6;
                        for (uint16_t i = 0; i < outputStreamLength; i++){
                            outputStream [i] = _receiveStream[i];
                        }
                        _receiveStreamLengthCtr = -1;
                        return receiveStreamType;
                    }
                }
            }
            _receiveStreamLengthCtr = -1;
        }
    }
    return 0;
}

void RTCMStreamInput::hashData(uint16_t startctr,uint16_t lengthctr)
{
    uint32_t crc = 0;
    for (int16_t i = 0; i < lengthctr; i++) {
	    crc = (crc<<8) ^ crc24q[(_receiveStream[i + startctr]) ^ (uint8_t)(crc >> 16)]; 
    }
    _inputStreamHash[0] = (crc >> 16) & 0xFF;
    _inputStreamHash[1] = (crc >> 8) & 0xFF;
    _inputStreamHash[2] = crc & 0xFF;
}
