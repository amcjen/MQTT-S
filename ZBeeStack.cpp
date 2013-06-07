/*
 * ZBeeStack.cpp
 *
 *               Copyright (c) 2009 Andrew Rapp.    All rights reserved.
 *               Copyright (c) 2013, tomy-tech.com  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 *     Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 *     Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation 
 *     and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * You should have received a copy of the GNU General Public License
 * If not, see <http://www.gnu.org/licenses/>.
 *
 *  Created on: 2013/05/24
 *     Author: Tomoaki YAMAGUCHI
 *     Version: 0.1
 *
 */
#ifndef ARDUINO
        #include "MQTTS_Defines.h"
#else
        #include <MQTTS_Defines.h>
#endif


#ifdef ARDUINO
  #include <ZBeeStack.h>

  #ifdef DEBUG_ZBEESTACK
  	#include <SoftwareSerial.h>
	extern SoftwareSerial debug;
  #endif

#endif  /* ARDUINO */

#ifdef MBED
	#include "mbed.h"
	#include "ZBeeStack.h"

    #ifdef DEBUG_ZBEESTACK
		    extern Serial debug;
        #endif
#endif /* MBED */

#ifdef LINUX
	#include "ZBeeStack.h"
	#include <stdio.h>
	#include <sys/time.h>
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <unistd.h>
	#include <stdlib.h>
	#include <string.h>
	#include <fcntl.h>
	#include <errno.h>
        #include <termios.h>

#endif /* LINUX */

using namespace std;

/*=========================================
             Class XBeeAddress64
 =========================================*/
XBeeAddress64::XBeeAddress64(){
  _msb = _lsb = 0;
}

XBeeAddress64::XBeeAddress64(uint32_t msb, uint32_t lsb){
  _msb = msb;
  _lsb = lsb;
}

uint32_t XBeeAddress64::getMsb(){
  return _msb;
}

uint32_t XBeeAddress64::getLsb(){
  return _lsb;
}

void XBeeAddress64::setMsb(uint32_t msb){
  _msb = msb;
}

void XBeeAddress64::setLsb(uint32_t lsb){
  _lsb = lsb;
}

/*=========================================
             Class XBeeResponse
 =========================================*/
XBeeResponse::XBeeResponse(){
  reset();
}

uint8_t XBeeResponse::getApiId(){
  return _apiId;
}

uint8_t XBeeResponse::getMsbLength(){
  return _msbLength;
}

uint8_t XBeeResponse::getLsbLength(){
  return _lsbLength;
}

uint8_t XBeeResponse::getChecksum(){
  return _checksum;
}

uint8_t XBeeResponse::getFrameDataLength(){
  return _frameLength;
}

uint8_t* XBeeResponse::getFrameData(){
  return _frameDataPtr;
}

uint16_t XBeeResponse::getPacketLength() {
        return ((_msbLength << 8) & 0xff) + (_lsbLength & 0xff);
}

void XBeeResponse::setApiId(uint8_t apiId){
  _apiId = apiId;
}

void XBeeResponse::setMsbLength(uint8_t msbLength){
  _msbLength = msbLength;
}

void XBeeResponse::setLsbLength(uint8_t lsbLength){
  _lsbLength = lsbLength;
}

void XBeeResponse::setChecksum(uint8_t checksum){
  _checksum = checksum;
}

void XBeeResponse::setFrameDataLength(uint8_t frameLength){
  _frameLength = frameLength;
}
void XBeeResponse::setFrameData(uint8_t *frameDataPtr){
  _frameDataPtr = frameDataPtr;
}

bool XBeeResponse::isAvailable(){
  return _complete;
}

void XBeeResponse::setAvailable(bool complete){
  _complete = complete;
}

bool XBeeResponse::isError(){
  return _errorCode > 0;
}

uint8_t XBeeResponse::getErrorCode(){
  return _errorCode;
}

void XBeeResponse::setErrorCode(uint8_t errorCode){
  _errorCode = errorCode;
}

void XBeeResponse::getZBRxResponse(XBeeResponse &rxResponse){
  ZBRxResponse *zr = static_cast<ZBRxResponse*>(&rxResponse);
  zr->setFrameData(getFrameData());
  copyCommon(rxResponse);
  zr->getRemoteAddress64().setMsb( (uint32_t)(
         (uint32_t(getFrameData()[0]) << 24) +
         (uint32_t(getFrameData()[1]) << 16) +
         (uint16_t(getFrameData()[2]) << 8)   +
             getFrameData()[3])
      );
  zr->getRemoteAddress64().setLsb( (uint32_t)(
           (uint32_t(getFrameData()[4]) << 24) +
           (uint32_t(getFrameData()[5]) << 16) +
           (uint16_t(getFrameData()[6]) << 8)   +
               getFrameData()[7])
        );
  copyCommon(rxResponse);
}

void XBeeResponse::getAtCommandResponse(XBeeResponse &atCommandResponse){
  AtCommandResponse *at = static_cast<AtCommandResponse*>(&atCommandResponse);
  at->setFrameData(getFrameData());
  at->setFrameId(getFrameData()[0]);
  at->setErrorCode(getFrameData()[3]);
  copyCommon(atCommandResponse);
}

void XBeeResponse::reset(){
  _apiId = 0;
  _msbLength = 0;
  _lsbLength = 0;
  _checksum = 0;
  _frameLength = 0;
  _errorCode = NO_ERROR;
  _complete = false;
}

/**
 *  private functions
 */
void XBeeResponse::copyCommon(XBeeResponse &target){
  target.setApiId(getApiId());
  target.setAvailable(isAvailable());
  target.setChecksum(getChecksum());
  target.setErrorCode(getErrorCode());
  target.setFrameDataLength(getFrameDataLength());
  target.setMsbLength(getMsbLength());
  target.setLsbLength(getLsbLength());
}

/*=========================================
             Class ZBRxResponse
 =========================================*/

ZBRxResponse::ZBRxResponse() : XBeeResponse(){
  _remoteAddress64 = XBeeAddress64();
}

uint8_t ZBRxResponse::getData(int index){
  return getFrameData()[getDataOffset() + index];
}

uint8_t* ZBRxResponse::getData(){
  return getFrameData() + getDataOffset();
}

uint8_t ZBRxResponse::getDataLength(){
  return getFrameDataLength() - getDataOffset();
}

XBeeAddress64&  ZBRxResponse::getRemoteAddress64(){
  return _remoteAddress64;
}


uint16_t ZBRxResponse::getRemoteAddress16(){
  return (getFrameData()[8] << 8) + getFrameData()[9];
}

uint8_t ZBRxResponse::getOption(){
  return getFrameData()[10];
}

uint8_t ZBRxResponse::getDataOffset(){
  return 11;
}

bool ZBRxResponse::isBroadcast(){
  return ( getOption() && 0x02);
}

/*=========================================
             Class FrameIdResponse
 =========================================*/

FrameIdResponse::FrameIdResponse(){
  _frameId = 0;
}

uint8_t FrameIdResponse::getFrameId(){
  return getFrameData()[0];
}

void FrameIdResponse::setFrameId(uint8_t frameId){
  _frameId = frameId;
}

/*=========================================
         Class AtCommandResponse
 =========================================*/

AtCommandResponse::AtCommandResponse(){

}

uint8_t* AtCommandResponse::getCommand(){
  return getFrameData() + 1;
}

uint8_t AtCommandResponse::getStatus(){
  return getFrameData()[3];
}

uint8_t AtCommandResponse::getValueLength(){
  return getFrameDataLength() - 4;
}

uint8_t* AtCommandResponse::getValue(){
  if (getValueLength() >0){
      return getFrameData() + 4;
  }else{
      return 0;
  }
}

bool AtCommandResponse::isOk(){
  return (getStatus() == AT_OK);
}

/*=========================================
           Class XBeeRequest
 =========================================*/

XBeeRequest::XBeeRequest(uint8_t apiId, uint8_t frameId){
  _apiId = apiId;
  _frameId = frameId;
}

void XBeeRequest::setFrameId(uint8_t frameId){
  _frameId = frameId;
}

uint8_t XBeeRequest::getFrameId(){
  return _frameId;
}

uint8_t XBeeRequest::getApiId(){
  return _apiId;
}

void XBeeRequest::setApiId(uint8_t apiId){
  _apiId = apiId;
}

/*=========================================
          Class AtCommandRequest
 =========================================*/

AtCommandRequest::AtCommandRequest() : XBeeRequest(AT_COMMAND_REQUEST, DEFAULT_FRAME_ID){
  _command = NULL;
  clearCommandValue();
}

AtCommandRequest::AtCommandRequest(uint8_t *command) : XBeeRequest(AT_COMMAND_REQUEST, DEFAULT_FRAME_ID){
  _command = command;
  clearCommandValue();
}

#ifndef ARDUINO
AtCommandRequest::~AtCommandRequest(){
  // avoid  compiler error
}
#endif

AtCommandRequest::AtCommandRequest(uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength) : XBeeRequest(AT_COMMAND_REQUEST, DEFAULT_FRAME_ID){
  _command = command;
  _commandValue = commandValue;
  _commandValueLength = commandValueLength;
}

uint8_t* AtCommandRequest::getCommand(){
  return _command;
}

uint8_t* AtCommandRequest::getCommandValue(){
  return _commandValue;
}

uint8_t AtCommandRequest::getCommandValueLength(){
  return _commandValueLength;
}

void AtCommandRequest::setCommand(uint8_t *command){
  _command = command;
}

void AtCommandRequest::setCommandValue(uint8_t *value){
  _commandValue = value;
}

void AtCommandRequest::setCommandValueLength(uint8_t length){
  _commandValueLength = length;
}

uint8_t AtCommandRequest::getFrameData(uint8_t pos){
  if (pos== 0){
     return getFrameId();
  }else if (pos == 1){
      return _command[0];
  }else if (pos == 2){
      return _command[1];
  }else{
      return _commandValue[pos - AT_COMMAND_API_LENGTH - 1];
  }
}

uint8_t AtCommandRequest::getFrameDataLength(){
  return AT_COMMAND_API_LENGTH + 1 + _commandValueLength;
}

void AtCommandRequest::clearCommandValue(){
  _commandValue = NULL;
  _commandValueLength = 0;
}

/*=========================================
             Class ZBTxRequest
 =========================================*/

ZBTxRequest::ZBTxRequest(XBeeAddress64 &addr64, uint8_t *payloadPtr, uint8_t payloadLength) : XBeeRequest(ZB_TX_REQUEST, DEFAULT_FRAME_ID) {
  init(addr64, (uint16_t)ZB_BROADCAST_ADDRESS, (uint8_t)ZB_BROADCAST_RADIUS_MAX_HOPS,
      (uint8_t)ZB_TX_UNICAST, payloadPtr, payloadLength);
}

ZBTxRequest::ZBTxRequest(XBeeAddress64 &addr64, uint16_t addr16, uint8_t broadcastRadius, uint8_t option,
        uint8_t *payloadPtr, uint8_t payloadLength, uint8_t frameId) : XBeeRequest(ZB_TX_REQUEST, frameId) {
  init(addr64, addr16, broadcastRadius, option, payloadPtr, payloadLength);
}

ZBTxRequest::ZBTxRequest() : XBeeRequest(ZB_TX_REQUEST, DEFAULT_FRAME_ID){
  init(XBeeAddress64(), (uint16_t)ZB_BROADCAST_ADDRESS, (uint8_t)ZB_BROADCAST_RADIUS_MAX_HOPS,
      (uint8_t)ZB_TX_UNICAST, NULL, 0);
}

ZBTxRequest::~ZBTxRequest(){

}

XBeeAddress64& ZBTxRequest::getAddress64(){
  return _addr64;
}

uint16_t ZBTxRequest::getAddress16(){
  return _addr16;
}

uint8_t ZBTxRequest::getBroadcastRadius(){
  return _broadcastRadius;
}

uint8_t  ZBTxRequest::getOption(){
  return _option;
}

uint8_t*  ZBTxRequest::getPayload(){
  return _payloadPtr;
}

uint8_t  ZBTxRequest::getPayloadLength(){
  return _payloadLength;
}

void ZBTxRequest::setAddress64(XBeeAddress64* addr64){
  _addr64.setMsb(addr64->getMsb());
  _addr64.setLsb(addr64->getLsb());
}

void ZBTxRequest::setAddress16(uint16_t addr16){
  _addr16 = addr16;
}

void ZBTxRequest::setBroadcastRadius(uint8_t broadcastRadius){
  _broadcastRadius = broadcastRadius;
}

void ZBTxRequest::setOption(uint8_t option){
  _option = option;
}

void ZBTxRequest::setPayload(uint8_t* payload){
  _payloadPtr = payload;
}

void ZBTxRequest::setPayloadLength(uint8_t payloadLength){
  _payloadLength = payloadLength;
}

uint8_t ZBTxRequest::getFrameData(uint8_t pos){
  if (pos == 0){
        return getFrameId();
    }else if (pos == 1){
        return (_addr64.getMsb() >> 24) & 0xff;
    }else if (pos == 2){
        return (_addr64.getMsb() >> 16) & 0xff;
    }else if (pos == 3){
        return (_addr64.getMsb() >> 8) & 0xff;
    }else if (pos == 4){
        return (_addr64.getMsb()) & 0xff;
    }else if (pos == 5){
        return (_addr64.getLsb() >> 24) & 0xff;
    }else if (pos == 6){
        return (_addr64.getLsb() >> 16) & 0xff;
    }else if (pos == 7){
        return (_addr64.getLsb() >> 8) & 0xff;
    }else if (pos == 8){
        return (_addr64.getLsb()) & 0xff;
    }else if (pos == 9){
        return (_addr16 >> 8) & 0xff;
    }else if (pos == 10){
        return _addr16 & 0xff;
    }else if (pos == 11){
        return _broadcastRadius;
    }else if (pos == 12){
        return _option;
    }
    return getPayload()[pos - ZB_TX_API_LENGTH -1 ];
}

uint8_t ZBTxRequest::getFrameDataLength(){
  return ZB_TX_API_LENGTH + 1 + getPayloadLength();
}

void ZBTxRequest::init(XBeeAddress64 addr64,uint16_t addr16, uint8_t broadcastRadius, uint8_t option, uint8_t* payloadPtr, uint8_t payloadLength){
    _addr64 = addr64;
    _addr16 = addr16;
    _broadcastRadius = broadcastRadius;
    _option = option;
    _payloadPtr = payloadPtr;
    _payloadLength = payloadLength;
}

/*=========================================
             Class XBee
 =========================================*/

XBee::XBee(){
  _response = XBeeResponse();
  _pos = 0;
  _escape = false;
  _checksumTotal = 0;
  _nextFrameId = 0;
  _response.setFrameData(_responseFrameData);
  _modemStatus = 0;
  _serialPort = 0;
  //_tm = XTimer();
  _tm.stop();
}

void XBee::getResponse(XBeeResponse &response){
  response.setMsbLength(_response.getMsbLength());
  response.setLsbLength(_response.getLsbLength());
  response.setApiId(_response.getApiId());
  response.setFrameDataLength(_response.getFrameDataLength());
  response.setFrameData(_response.getFrameData());
}

XBeeResponse& XBee::getResponse(){
  return _response;
}

void XBee::readApiFrame(){

  if(_response.isAvailable() || _response.isError()){
    resetResponse();
  }

  while(read(_b )){
      // Check Start Byte
      if( _pos > 0 && _b[0] == START_BYTE){
          _response.setErrorCode(UNEXPECTED_START_BYTE);
          return;
      }
      // Check ESC
      if(_pos > 0 && _b[0] == ESCAPE){
          if(read(_b )){
              _b[0] = 0x20^_b[0];  // decode
          }else{
              _escape = true;
              continue;
          }
      }

      if(_escape){
          _b[0] = 0x20 ^ _b[0];
          _escape = false;
      }

      if(_pos >= API_ID_INDEX){
          _checksumTotal+= _b[0];
      }
      switch(_pos){
        case 0:
          if(_b[0] == START_BYTE){
              _pos++;
          }
          break;
        case 1:
          _response.setMsbLength(_b[0]);
          _pos++;
          break;
        case 2:
          _response.setLsbLength(_b[0]);
          _pos++;
          break;
        case 3:
          _response.setApiId(_b[0]);
          _pos++;
          break;
        default:
          if(_pos > MAX_FRAME_DATA_SIZE){
              _response.setErrorCode(UNEXPECTED_START_BYTE);
              return;
          }else if(_pos == (_response.getPacketLength() + 3)){  // 3 = 2(packet len) + 1(checksum)
              if((_checksumTotal & 0xff) == 0xff){
                  _response.setChecksum(_b[0]);
                  _response.setAvailable(true);
                  _response.setErrorCode(NO_ERROR);
              }else{
                  _response.setErrorCode(CHECKSUM_FAILURE);
              }
              _response.setFrameDataLength(_pos - 4);    // 4 = 2(packet len) + 1(Api) + 1(checksum)
              _pos = 0;
              _checksumTotal = 0;
              return;
          }else{
              uint8_t* buf = _response.getFrameData();
              buf[_pos - 4] = _b[0];
              _pos++;
          }
          break;
      }
  }
}

bool XBee::readApiFrame(long timeoutMillsec){
  _tm.start(timeoutMillsec);
  while(!_tm.isTimeUp()){
      readApiFrame();

      if(getResponse().isAvailable()){
            #ifdef DEBUG_ZBEESTACK
                #ifdef ARDUINO
                    debug.println("");
                #endif
                    #ifdef MBED
                    debug.fprintf(stdout,"\n" );
                    #endif
                    #ifdef LINUX
                    fprintf(stdout,"\n" );
                #endif
            #endif
          return true;
      }else if(getResponse().isError()){
            #ifdef DEBUG_ZBEESTACK
                #ifdef ARDUINO
                    debug.println("");
                #endif
                    #ifdef MBED
                    debug.fprintf(stdout,"\n" );
                    #endif
                    #ifdef LINUX
                    fprintf(stdout,"\n" );
                #endif
            #endif
          return false;
      }
  }
  return false;   //Timeout
}

uint8_t XBee::getNextFrameId(){
  _nextFrameId++;
    if (_nextFrameId == 0){
        _nextFrameId = 1;
    }
    return _nextFrameId;
}

uint8_t XBee::getModemStatus(){
  return _modemStatus;
}

void XBee::setModemStatus(uint8_t modemStatus){
  _modemStatus = modemStatus;
}

void XBee::setSerialPort(SerialPort *serialPort){
  _serialPort = serialPort;
}

void XBee::send(XBeeRequest &request){
  sendByte(START_BYTE, false);

  uint8_t msbLen = ((request.getFrameDataLength() + 1) >> 8) & 0xff; // 1 = 1B(Api)  except Checksum
  uint8_t lsbLen = (request.getFrameDataLength() + 1) & 0xff;
  sendByte(msbLen, true);
  sendByte(lsbLen, true);

  sendByte(request.getApiId(), true);
  uint8_t checksum = 0;
  checksum+= request.getApiId();

  for( int i = 0; i < request.getFrameDataLength(); i++ ){
      sendByte(request.getFrameData(i), true);
      checksum+= request.getFrameData(i);
  }
  checksum = 0xff - checksum;
  sendByte(checksum, true);

  flush();  // clear receive buffer

#ifdef DEBUG_ZBEESTACK
    #ifdef ARDUINO
        debug.println("");
    #endif
	#ifdef MBED
        debug.fprintf(stdout,"\n" );
	#endif
	#ifdef LINUX
        fprintf(stdout,"\n" );
    #endif
#endif
}

void XBee::sendByte(uint8_t b, bool escape){
  if(escape && (b == START_BYTE || b == ESCAPE || b == XON || b == XOFF)){
      write(ESCAPE);
      write(b ^ 0x20);
  }else{
      write(b);
  }
}

void XBee::resetResponse(){
  _pos = 0;
  _escape = 0;
  _response.reset();
}

void XBee::flush(){
  _serialPort->flush();
}

bool XBee::write(uint8_t val){
  return (_serialPort->send(val) ? true : false );
}

bool XBee::read(uint8_t *buff){
        return  _serialPort->recv(buff);
}

/*=========================================
             Class XBeeTimer
 =========================================*/

#ifndef ARDUINO
XTimer::XTimer(){
  stop();
}

void XTimer::start(long msec){
  gettimeofday(&_startTime, NULL);
  _millis = msec;
}

bool XTimer::isTimeUp(){
  struct timeval curTime;
    long int secs, usecs;

    if (_startTime.tv_sec == 0){
        return false;
    }else{
    gettimeofday(&curTime, NULL);
        secs  = curTime.tv_sec  - _startTime.tv_sec;
        usecs = curTime.tv_usec - _startTime.tv_usec;
        return ((uint16_t)((secs) * 1000 + usecs/1000.0) > _millis);
    }
}

void XTimer::start(){
  gettimeofday(&_startTime, NULL);
  _millis = 0;
}

bool XTimer::isTimeUp(long msec){
  struct timeval curTime;
    long int secs, usecs;
    if (_startTime.tv_sec == 0){
        return false;
    }else{
        gettimeofday(&curTime, NULL);
        secs  = curTime.tv_sec  - _startTime.tv_sec;
        usecs = curTime.tv_usec - _startTime.tv_usec;
        return ((uint16_t)((secs) * 1000 + usecs/1000.0) > msec);
    }
}

void XTimer::stop(){
  _startTime.tv_sec = 0;
  _millis = 0;
}


#else
/**
 *   for Arduino
 */
XTimer::XTimer(){
    stop();
}

void XTimer::start(long msec){
    _startTime = millis();
    _millis = msec;
}

bool XTimer::isTimeUp(){
    if (_millis == 0){
        return false;
    }else{
        return (((uint16_t)millis() - _startTime) > _millis);
    }
}

void XTimer::start(){
  _startTime = millis();
  _millis = 0;
}

bool XTimer::isTimeUp(long msec){
    if ( _startTime == 0){
        return false;
    }else{
        return (((uint16_t)millis() - _startTime) > msec);
    }
}

void XTimer::stop(){
    _startTime = 0;
    _millis = 0;
}

#endif  /* ARDUINO */

/*=========================================
       Class SerialPort
 =========================================*/
#ifdef ARDUINO
/**
 *  For Arduino
 */
SerialPort::SerialPort(){
  _serial = NULL;
}

void SerialPort::begin(long baudrate){
  Serial.begin(baudrate);
  _serial = (Stream*) &Serial;
}

bool SerialPort::send(unsigned char b){
  if(_serial->write(b) != 1){
      return false;
  }else{
#ifdef DEBUG_ZBEESTACK
	debug.print(" s:0x");
        debug.print(b,HEX);
#endif
      return true;
  }
}

bool SerialPort::recv(unsigned char* buf){
  if ( _serial->available() > 0 ){
    buf[0] = _serial->read();
#ifdef DEBUG_ZBEESTACK
	debug.print(" r:0x");
        debug.print(buf[0],HEX);
#endif
    return true;
  }else{
    return false;
  }
}

void SerialPort::flush(void){
  _serial->flush();
}

#endif /* ARDUINO */

#ifdef MBED
/**
 *  For MBED
 */
SerialPort::SerialPort(){
  _serial = new Serial(ZB_MBED_SERIAL_TXPIN, ZB_MBED_RXPIN);
}

void SerialPort::begin(long baudrate){
  _serial->baud(baudrate);
  _serial->format(8,Serial::None,1);
}

bool SerialPort::send(unsigned char b){
  _serial->putc(b);
#ifdef DEBUG_ZBEESTACK
	debug.fprintf(stdout, " S:0x%x", b);
#endif
      return true;
  }
}

bool SerialPort::recv(unsigned char* buf){
  if ( _serial->readable() > 0 ){
    buf[0] = _serial->getc();
#ifdef DEBUG_ZBEESTACK
        debug.fprintf(stdout, " R:0x%x",buf[0] );
#endif
    return true;
  }else{
    return false;
  }
}

void SerialPort::flush(void){
  _serial->flush();
}

#endif /* MBED */

#ifdef LINUX

SerialPort::SerialPort(){
    _tio.c_iflag = IGNBRK | IGNPAR;
#ifdef XBEE_FLOWCTRL_CRTSCTS
    _tio.c_cflag = CS8 | CLOCAL | CREAD | CRTSCTS;
#else
    _tio.c_cflag = CS8 | CLOCAL | CREAD;
#endif
    _tio.c_cc[VINTR] = 0;
    _tio.c_cc[VTIME] = 0;
    _tio.c_cc[VMIN] = 0;
    _fd = 0;
}

SerialPort::~SerialPort(){
  if (_fd){
      close(_fd);
  }
}

int SerialPort::begin(const char* devName){
  return begin(devName, B9600, false, 1);
}


int SerialPort::begin(const char* devName, unsigned int boaurate){
  return begin(devName, boaurate, false, 1);
}

int SerialPort::begin(const char* devName, unsigned int boaurate, bool parity){
  return begin(devName, boaurate, parity, 1);
}

int SerialPort::begin(const char* devName, unsigned int boaurate,  bool parity, unsigned int stopbit){
  _fd = open(devName, O_RDWR | O_NOCTTY);
  if(_fd < 0){
      return _fd;
  }

  if (parity){
      _tio.c_cflag = _tio.c_cflag | PARENB;
  }
  if (stopbit == 2){
      _tio.c_cflag = _tio.c_cflag | CSTOPB ;
  }
  switch(boaurate){
    case B9600:
    case B19200:
    case B38400:
    case B57600:
    case B115200:
      if( cfsetspeed(&_tio, boaurate)<0){
        return errno;
      }
      break;
    default:
      return -1;
  }
    return tcsetattr(_fd, TCSANOW, &_tio);
}

bool SerialPort::send(unsigned char b){
  if (write(_fd, &b,1) != 1){
      return false;
  }else{
      #ifdef DEBUG_ZBEESTACK
          fprintf(stdout, " S:0x%x", b);
      #endif
      return true;
  }
}

bool SerialPort::recv(unsigned char* buf){
  if(read(_fd, buf, 1) == 0){
      return false;
  }else{
      #ifdef DEBUG_ZBEESTACK
          fprintf(stdout, " R:0x%x",buf[0] );
      #endif
      return true;
  }
}

void SerialPort::flush(void){
  tcsetattr(_fd, TCSAFLUSH, &_tio);
}
#endif  /* LINUX */


/*===========================================
              Class  ZBeeStack
 ============================================*/

ZBeeStack::ZBeeStack(){
 _respWaitStat = ZB_WAIT_NORESP;
 _sendReqStat = ZB_SEND_REQ_NO;
 _rxDataReady = false;
 _readTimeout = PACKET_TIMEOUT;
 _rxCallbackPtr = NULL;
 _rxData = NULL;
 _returnCode = 0;
}

void ZBeeStack::setReceivePacketTimeout(long timeout){
  _readTimeout = timeout;
}

bool ZBeeStack::setNodeId(const char* id){
  int len = strlen(id);
  if (len <= ZB_MAX_NODEID ){
          uint8_t node[len + 1];
          strcpy((char*)node, id);
          if (sendAt("NI", node, len ) == 0){
                  return true;
          }else{
                  return false;
          }
  }else{
          return false;
  }
}

ZBNode ZBeeStack::getZBNode(){
    return _zbNode;
}

const char* ZBeeStack::getNodeId(){
  return getZBNode().getNodeId();
}

int ZBeeStack::sendAt(const char* cmd){
  return sendAt(cmd, NULL, 0); // PACKET_ERROR_TIMEOUT, PACKET_ERROR_RESPONSE, PACKET_CORRECT
}

int ZBeeStack::sendAt(const char* cmd, uint8_t* cmdValue, uint8_t valLength){
  char command[AT_COMMAND_API_LENGTH + 1];
  strncpy(command, cmd, AT_COMMAND_API_LENGTH);
  command[AT_COMMAND_API_LENGTH] = 0;
  _atRequest.setCommand((uint8_t*)command);
  _atRequest.setCommandValue(cmdValue);
  _atRequest.setCommandValueLength(valLength);
  _atRequest.setFrameId(_xbee.getNextFrameId());
  _sendReqStat = ZB_SEND_REQ_AT;
  _respWaitStat = ZB_WAIT_NORESP;
  return packetHandle(); // PACKET_ERROR_TIMEOUT, PACKET_ERROR_RESPONSE, PACKET_CORRECT
}


int ZBeeStack::sendData(XBeeAddress64* addr64, uint16_t addr16, uint8_t* payload,
                           uint8_t payloadLen, uint8_t option ){
  _txRequest.setAddress64(addr64);
  _txRequest.setAddress16(addr16);
  _txRequest.setOption(option);
  _txRequest.setPayload(payload);
  _txRequest.setPayloadLength(payloadLen);
  _txRequest.setFrameId(0);                // Stop to Replay
  _sendReqStat = ZB_SEND_REQ_TX;
  _respWaitStat = ZB_WAIT_NORESP;
  return packetHandle();
}

int ZBeeStack::bcastData(uint8_t* xmitData, uint8_t dataLen){
  XBeeAddress64 addr;
  addr.setLsb(BROADCAST_ADDRESS);
  return sendData(&addr, ZB_BROADCAST_ADDRESS, xmitData, dataLen, 0);
}

int ZBeeStack::readPacket(){
  _sendReqStat = ZB_SEND_REQ_NO;
  _respWaitStat = ZB_WAIT_NORESP;
  return packetHandle();
}

int ZBeeStack::readResp(){
  _sendReqStat = ZB_SEND_REQ_NO;
  _respWaitStat = ZB_WAIT_RX_RESP;
  return packetHandle();
}


void ZBeeStack::setRxHandler(void (*callbackPtr)(ZBRxResponse* data, int* returnCode)){
 _rxCallbackPtr = callbackPtr;
}

void ZBeeStack::setSerialPort(SerialPort* sp){
 _xbee.setSerialPort(sp);
}

XBeeAddress64& ZBeeStack::getRxRemoteAddress64(){
  return _rxData->getRemoteAddress64();
}

uint16_t ZBeeStack::getRxRemoteAddress16(){
  return _rxData->getRemoteAddress16();
}

AtCommandResponse*  ZBeeStack::getAtResponse(){
  return &_atResp;
}

ZBRxResponse* ZBeeStack::getRxResponse(){
  return &_rxResp;
}

ZBRxResponse* ZBeeStack::getRxData(){
  return _rxData;
}

/*----------------------------------------
 *          via XBee
 -----------------------------------------*/
int ZBeeStack::packetHandle(){
    while(true){
        if (_respWaitStat == ZB_WAIT_NORESP){
            if (_sendReqStat == ZB_SEND_REQ_NO && !_xbee.readApiFrame(_readTimeout)){
                return PACKET_ERROR_NODATA;
            }
        }else{
            if (!_xbee.readApiFrame(_readTimeout)){
                return PACKET_ERROR_NODATA;
            }
        }

        switch(_xbee.getResponse().getApiId()){
        case ZB_RX_RESPONSE:
            if (_respWaitStat == ZB_WAIT_NORESP || _respWaitStat == ZB_WAIT_RX_RESP) {
                _xbee.getResponse().getZBRxResponse(_rxResp);
                if (_rxResp.isError()){           // received packet is not collect
                    return PACKET_ERROR_RESPONSE;
                }else{    //  Copy Data
                    for ( int i = 0; i < _rxResp.getFrameDataLength(); i++){
                        _rxDataBuf[i] = _rxResp.getFrameData()[i];
                    }
                    _rxResp.setFrameData(_rxDataBuf);
                    _rxData = &_rxResp;
                    _rxDataReady = true;
                }
            }
            break;

        case AT_COMMAND_RESPONSE:
            if (_respWaitStat == ZB_WAIT_AT_RESP ) {
               _xbee.getResponse().getAtCommandResponse(_atResp);
               if ((_atResp.getFrameId() == _atRequest.getFrameId())
                    && _atResp.getCommand()[0] == _atRequest.getCommand()[0]
                    && _atResp.getCommand()[1] == _atRequest.getCommand()[1] ){
                   if (_atResp.isOk()){
                       _respWaitStat = ZB_WAIT_NORESP;
                       return PACKET_CORRECT;
                   }else{
                       _respWaitStat = ZB_WAIT_NORESP;
                       return PACKET_ERROR_RESPONSE;
                   }
                }   // FrameId or Command error
             }
             break;

        default:
            break;
        }

        /* ============= Send Request  ============== */
        if (_respWaitStat == ZB_WAIT_NORESP) {
            switch (_sendReqStat) {
            case ZB_SEND_REQ_NO:
                if ( _rxDataReady && (_rxCallbackPtr != NULL)){
                    _rxCallbackPtr(getRxData(), &_returnCode);
                    _rxDataReady = false;
                    return _returnCode;
                }
                return PACKET_ERROR_NODATA;
                break;

            case ZB_SEND_REQ_TX:
                _xbee.send(_txRequest);
                return PACKET_CORRECT;
                break;

            case ZB_SEND_REQ_AT:
                _respWaitStat = ZB_WAIT_AT_RESP;
                _sendReqStat = ZB_SEND_REQ_NO;
                _xbee.send(_atRequest);
                break;

            default:
                return PACKET_ERROR_UNKOWN;
                break;
            }
        }

        if (_rxDataReady && (_rxCallbackPtr != NULL)){
            _rxCallbackPtr(getRxData(), &_returnCode);
            _rxDataReady = false;
        }

    }
    return -1;  // for compiler
}


bool ZBeeStack::init(ZBNodeType type, const char* nodeId){
  uint16_t a16 = 0;
  uint32_t msb = 0;
  uint32_t lsb = 0;

  if (sendAt("MY") == PACKET_CORRECT){
      uint8_t* value = getAtResponse()->getValue();
      a16 = (a16 + (uint32_t)value[0]) << 8;
      a16 = (a16 + (uint32_t)value[1]);
      _zbNode.setAddress16(a16);
  }

  if (sendAt("SH") == PACKET_CORRECT){
      uint8_t* value = getAtResponse()->getValue();

      msb = (msb + (uint32_t)value[0]) << 8;
      msb = (msb + (uint32_t)value[1]) << 8;
      msb = (msb + (uint32_t)value[2]) << 8;
      msb = (msb + (uint32_t)value[3]);
  }
  if (sendAt("SL") == PACKET_CORRECT){
      uint8_t* value = getAtResponse()->getValue();
      lsb = (lsb + (uint32_t)value[0]) << 8;
      lsb = (lsb + (uint32_t)value[1]) << 8;
      lsb = (lsb + (uint32_t)value[2]) << 8;
      lsb = (lsb + (uint32_t)value[3]);
      _zbNode.setMsb(msb);
      _zbNode.setLsb(lsb);
  }
  _zbNode.setNodeId(nodeId);
  setNodeId(nodeId);

  _zbNode.setDeviceType(type);
  _zbNode.setNodeStatus(MQTTS_DEVICE_DISCONNECTED);
  return true;
}


/*===========================================
              Class  ZBNode
 ============================================*/

ZBNode::ZBNode() {
  _address64 = XBeeAddress64();
  _address16 = 0;
  _deviceType = 0;
  _nodeId[0] = 0;
  _nodeStatus = 0;
}

XBeeAddress64* ZBNode::getAddress64(){
  return &_address64;
}

uint16_t ZBNode::getAddress16(){
  return _address16;
}

uint8_t ZBNode::getNodestatus(){
  return _nodeStatus;
}

char* ZBNode::getNodeId(){
  return _nodeId;
}
void ZBNode::setMsb(uint32_t msb){
  _address64.setMsb(msb);
}

void ZBNode::setLsb(uint32_t lsb){
  _address64.setLsb(lsb);
}

void ZBNode::setAddress16(uint16_t addr){
  _address16 = addr;
}

void ZBNode::setNodeStatus(uint8_t stat){
  _nodeStatus = stat;
}

void ZBNode::setNodeId(const char* id){
  if (id != 0 ){
      strcpy(_nodeId, id);
  }
}

void ZBNode::setDeviceType(uint8_t type){
  _deviceType = type;
}

bool ZBNode::isGateway(){
  return (_deviceType == 2);
}

bool ZBNode::isClient(){
  return (_deviceType == 3);
}

bool ZBNode::isCoordinator(){
  return (_deviceType == 1);
}
