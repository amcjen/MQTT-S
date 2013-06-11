/*
 * ZBeeStack.h
 *
 *
 *               Copyright (c) 2009 Andrew Rapp.    All rights reserved.
 *               Copyright (c) 2013 tomy-tech.com  All rights reserved.
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
 * 
 *  Created on: 2013/06/11
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 0.4.0
 *
 */

#ifndef ZBEESTACK_H_
#define ZBEESTACK_H_

#ifndef ARDUINO
        #include "MQTTS_Defines.h"
#else
        #include <MQTTS_Defines.h>
#endif

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
	#include <inttypes.h>
#else
	#if defined(ARDUINO) && ARDUINO < 100
		#include "WProgram.h"
		#include <inttypes.h>
    #endif
#endif /* ARDUINO */

#ifdef MBED
	#include "mbed.h"

	#define ZB_MBED_SERIAL_TXPIN  ()
	#define ZB_MBED_SERIAL_RXPIN  ()
#endif

#ifdef LINUX
	#include <sys/time.h>
	#include <iostream>
#endif

#define START_BYTE 0x7e
#define ESCAPE     0x7d
#define XON        0x11
#define XOFF       0x13

#define MAX_FRAME_DATA_SIZE  128

#define BROADCAST_ADDRESS    0xffff
#define ZB_BROADCAST_ADDRESS 0xfffe

#define DEFAULT_FRAME_ID 1

#define ZB_PACKET_ACKNOWLEGED  0x01
#define ZB_BROADCAST_PACKET    0x02
#define ZB_BROADCAST_RADIUS_MAX_HOPS 0

#define API_ID_INDEX  3
#define PACKET_OVERHEAD_LENGTH 6
#define ZB_TX_API_LENGTH  12
#define AT_COMMAND_API_LENGTH 2

/**
 * API ID Constant
 */
#define AT_COMMAND_REQUEST     0x08
#define ZB_TX_REQUEST          0x10
#define AT_COMMAND_RESPONSE    0x88
#define MODEM_STATUS_RESPONSE  0x8a
#define ZB_TX_STATUS_RESPONSE  0x8b
#define ZB_RX_RESPONSE         0x90
#define NODE_ID_IDENTIFIER     0x95

/**
 * TX STATUS
 */
#define SUCCESS           0x0

#define AT_OK                0
#define AT_ERROR             1
#define AT_INVALID_COMMAND   2
#define AT_INVALID_PARAMETER 3
#define AT_NO_RESPONSE       4

#define NO_ERROR                          0
#define CHECKSUM_FAILURE                  1
#define PACKET_EXCEEDS_BYTE_ARRAY_LENGTH  2
#define UNEXPECTED_START_BYTE             3

// modem status
#define HARDWARE_RESET 0
#define WATCHDOG_TIMER_RESET 1
#define ASSOCIATED 2
#define DISASSOCIATED 3
#define SYNCHRINIZATION_LOST 4
#define COORDINATOR_REALIGNMENT 5
#define COORDINATOR_STARTED 6

#define ZB_TX_UNICAST 0
#define ZB_TX_BROADCAST 8

#define ZB_MAX_NODEID 20

/*
 *   NodeType
 */
typedef  uint8_t ZBNodeType;
#define  ZB_COORDINATOR  1
#define  ZB_GATEWAY      2
#define  ZB_CLIENT       3


#define  PACKET_CORRECT          0
#define  PACKET_ERROR_RESPONSE   -1
#define  PACKET_ERROR_UNKOWN    -2
#define  PACKET_ERROR_NODATA     -3

#define  ZB_WAIT_NORESP    0
#define  ZB_WAIT_AT_RESP   1
#define  ZB_WAIT_RX_RESP   2

#define  ZB_SEND_REQ_NO    0
#define  ZB_SEND_REQ_AT    1
#define  ZB_SEND_REQ_TX    2

/*
 *   Packet Read Constants
 */
#ifdef ARDUINO
#define PACKET_TIMEOUT 100
#define PACKET_TIME_OUT_RESP  1000
#else
#define PACKET_TIME_OUT_RESP  1000
#define PACKET_TIMEOUT 100
#endif

/*
 *   MQTTS  Client's state
 */
#define MQTTS_DEVICE_DISCONNECTED     0
#define MQTTS_DEVICE_ACTIVE           1
#define MQTTS_DEVICE_ASLEEP           2
#define MQTTS_DEVICE_AWAKE            3
#define MQTTS_DEVICE_LOST             4


/*============================================
              XBeeAddress64
 =============================================*/
class XBeeAddress64 {
public:
  XBeeAddress64(uint32_t msb, uint32_t lsb);
  XBeeAddress64();
  uint32_t getMsb();
  uint32_t getLsb();
  void setMsb(uint32_t msb);
  void setLsb(uint32_t lsb);
private:
  uint32_t _msb;
  uint32_t _lsb;
};

 /*============================================
                XBeeResponse
 =============================================*/

class XBeeResponse {
public:
  XBeeResponse();
  uint8_t getApiId();
  uint8_t getMsbLength();
  uint8_t getLsbLength();
  uint8_t getChecksum();
  uint8_t getFrameDataLength();
  uint8_t* getFrameData();
  uint16_t getPacketLength();

  void setApiId(uint8_t apiId);
  void setMsbLength(uint8_t msbLength);
  void setLsbLength(uint8_t lsbLength);
  void setChecksum(uint8_t checksum);
  void setFrameData(uint8_t *frameDataPtr);
  void setFrameDataLength(uint8_t frameLength);

  void getZBRxResponse(XBeeResponse &response);
  void getAtCommandResponse(XBeeResponse &response);

  bool isAvailable();
  void setAvailable(bool complete);
  bool isError();
  uint8_t getErrorCode();
  void setErrorCode(uint8_t errorCode);
  void reset();

protected:
  uint8_t *_frameDataPtr;
private:
  void copyCommon(XBeeResponse &target);
  uint8_t _apiId;
  uint8_t _msbLength;
  uint8_t _lsbLength;
  uint8_t _checksum;
  uint8_t _frameLength;
  bool _complete;
  uint8_t _errorCode;
};

/*============================================
                ZBRxResponse
 =============================================*/


class ZBRxResponse : public XBeeResponse {
public:
  ZBRxResponse();
  XBeeAddress64& getRemoteAddress64();
  uint16_t getRemoteAddress16();
  uint8_t getOption();
  uint8_t getData(int index);
  uint8_t* getData();

  uint8_t getDataLength();
  uint8_t getDataOffset();
  bool   isBroadcast();

private:
  XBeeAddress64 _remoteAddress64;
};

/*============================================
                FrameIdResponse
 =============================================*/

class FrameIdResponse : public XBeeResponse {
public:
  FrameIdResponse();
  uint8_t getFrameId();
  void setFrameId(uint8_t frameId);
private:
  uint8_t _frameId;
};

/*============================================
 *               AtCommandResponse
 ============================================*/

class AtCommandResponse : public FrameIdResponse {
public:
  AtCommandResponse();
  uint8_t* getCommand();
  uint8_t getStatus();
  uint8_t* getValue();
  uint8_t getValueLength();
  bool isOk();
};

/*============================================*
                XBeeRequest
 =============================================*/

class XBeeRequest {
public:
  XBeeRequest(uint8_t apiId, uint8_t frameId);
  virtual ~XBeeRequest(){};
  void setFrameId(uint8_t frameId);
  uint8_t getFrameId();
  uint8_t getApiId();
  virtual uint8_t getFrameData(uint8_t pos) = 0;
  virtual uint8_t getFrameDataLength() = 0;
protected:
  void setApiId(uint8_t apiId);
private:
  uint8_t _apiId;
  uint8_t _frameId;
};

/*============================================
                AtCommandRequest
 =============================================*/

class AtCommandRequest : public XBeeRequest {
public:
  AtCommandRequest();
  AtCommandRequest(uint8_t *command);
  AtCommandRequest(uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength);
#ifndef ARDUINO
  virtual ~AtCommandRequest();
#endif
  uint8_t getFrameData(uint8_t pos);
  uint8_t getFrameDataLength();
  uint8_t* getCommand();
  uint8_t* getCommandValue();
  uint8_t getCommandValueLength();
  void setCommand(uint8_t *command);
  void setCommandValue(uint8_t *value);
  void setCommandValueLength(uint8_t length);
  void clearCommandValue();
private:
  uint8_t *_command;
  uint8_t *_commandValue;
  uint8_t _commandValueLength;
};

/*============================================
                 ZBTxRequest
 =============================================*/

class ZBTxRequest : public XBeeRequest{
public:
  ZBTxRequest();
  ZBTxRequest(XBeeAddress64 &addr64, uint8_t *payload, uint8_t payLoadLength);
  ZBTxRequest(XBeeAddress64 &addr64, uint16_t addr16, uint8_t broadcastRadius,
                uint8_t option, uint8_t *payload, uint8_t payloadLength, uint8_t frameId);
  virtual ~ZBTxRequest();
  XBeeAddress64& getAddress64();
  uint16_t getAddress16();
  uint8_t getBroadcastRadius();
  uint8_t getOption();
  uint8_t* getPayload();
  uint8_t getPayloadLength();

  void setAddress64(XBeeAddress64* addr64);
  void setAddress16(uint16_t addr16);
  void setBroadcastRadius(uint8_t broadcastRadius);
  void setOption(uint8_t option);
  void setPayload(uint8_t *payload);
  void setPayloadLength(uint8_t payLoadLength);
  uint8_t getFrameData(uint8_t pos);
  uint8_t getFrameDataLength();
private:
  void init(XBeeAddress64 addr64,uint16_t addr16, uint8_t broadcastRadius, uint8_t option, uint8_t* payloadPtr, uint8_t payloadLength);
  XBeeAddress64 _addr64;
  uint16_t _addr16;
  uint8_t _broadcastRadius;
  uint8_t _option;
  uint8_t* _payloadPtr;
  uint8_t _payloadLength;
};

/*===========================================
                SerialPort
 ============================================*/

#ifdef ARDUINO
#include <Stream.h>
class SerialPort{
public:
        SerialPort( );
        void begin(long baudrate);
        bool send(unsigned char b);
        bool recv(unsigned char* b);
        void flush();

private:
        Stream* _serial;
};
#endif /* ARDUINO */

#ifdef MBED
/*-------------------------
    For MBED
 --------------------------*/
class SerialPort{
public:
        SerialPort( );
        void begin(long baudrate);
        bool send(unsigned char b);
        bool recv(unsigned char* b);
        void flush();

private:
        Serial* _serial;
};
#endif /* MBED */

#ifdef LINUX
/*-------------------------
    For Linux
 --------------------------*/
#include <termios.h>
class SerialPort{
public:
        SerialPort();
        ~SerialPort();
        int begin(const char* devName);
        int begin(const char* devName,
                   unsigned int boaurate);
        int begin(const char* devName, unsigned int boaurate, bool parity);
        int begin(const char* devName, unsigned int boaurate,
                      bool parity, unsigned int stopbit);

        bool send(unsigned char b);
        bool recv(unsigned char* b);
        void flush();

private:
        int _fd;  // file descriptor
        struct termios _tio;
};
#endif /* LINUX */




/*============================================
                XBeeTimer
 ============================================*/
class XTimer {
public:
  XTimer();
  void start(long msec);
  void start(void);
  bool isTimeUp(long msec);
  bool isTimeUp(void);
  void stop();
private:

#ifdef ARDUINO
  long _startTime;
#else
  struct timeval _startTime;
#endif
  long _millis;
};


/*============================================
                 XBee
 ============================================*/

class XBee {
public:
  XBee();
  //void readPacket(void);
  void readApiFrame(void);
  bool readApiFrame(long timeoutMillsec);
  void getResponse(XBeeResponse &response);
  uint8_t getModemStatus();
  void setModemStatus(uint8_t);
  XBeeResponse& getResponse(void);
  void send(XBeeRequest &request);
  uint8_t getNextFrameId(void);
  void setSerialPort(SerialPort *serial);
  void flush();
private:
  void resetResponse();
  bool read(uint8_t* buff);
  bool write(uint8_t val);
  void sendByte(uint8_t, bool escape);
  XBeeResponse _response;
  bool _escape;
  uint8_t _pos;
  uint8_t _checksumTotal;
  uint8_t _nextFrameId;
  uint8_t _responseFrameData[MAX_FRAME_DATA_SIZE];
  SerialPort *_serialPort;
  uint8_t _modemStatus;
  uint8_t _b[1];
  XTimer   _tm;
};


/*===========================================
               Class  ZBNode
 ============================================*/
class ZBNode {
public:
    ZBNode();
    XBeeAddress64* getAddress64();
    uint16_t  getAddress16();
    uint8_t   getNodestatus();
    char*     getNodeId();
    void      setMsb(uint32_t msb);
    void      setLsb(uint32_t lsb);
    void      setAddress16(uint16_t addr);
    void      setNodeStatus(uint8_t stat);
    void      setNodeId(const char* id);
    void      setDeviceType(uint8_t type);
    bool      isGateway();
    bool      isClient();
    bool      isCoordinator();

private:
    XBeeAddress64 _address64;
    uint16_t  _address16;
    char     _nodeId[ZB_MAX_NODEID + 1];
    uint8_t   _nodeStatus;    // 0:init  1:Gw Active  3:Disconn  4:Active  5:Asleep  6:Awake
    uint8_t   _deviceType;    // 1:Cordinator   2:Gateway  3:Client
};

/*===========================================
               Class  ZBNodeList
 ============================================*/
class ZBNodeList {
public:
  ZBNodeList(uint16_t length = 5);
  ZBNode*   getZBNode(char* id);
  ZBNode*   getPrimaryGateway();
  ZBNode*   getSecondaryGateway();
  ZBNode*   getZBNode(XBeeAddress64 addr);
  ZBNode*   getZBNode(uint16_t addr);
  bool     addNodeDevice(ZBNode *nd);
  bool     deleteNodeDevise(const char* ndeId);
private:
  ZBNode**  _nodeList;
  uint8_t  _nodeListLength;
};


/*===========================================
               Class  ZBeeStack
 ============================================*/
class ZBeeStack {
public:
  ZBeeStack();
  int     sendAt(const char* cmd);
  int     sendAt(const char* cmd, uint8_t* cmdValue, uint8_t valueLenght);
  int     sendData(XBeeAddress64* addr64, uint16_t addr16, uint8_t* xmitData, uint8_t dataLen,
                    uint8_t flg = ZB_TX_UNICAST);
  int     bcastData(uint8_t* xmitData, uint8_t dataLen);
  void    setRxHandler(void (*callbackPtr)(ZBRxResponse* data, int* returnCode));
  void    setSerialPort(SerialPort *serialPort);
  int     readPacket();
  int     readResp();
  void    setReceivePacketTimeout(long timeout);
  XBeeAddress64& getRxRemoteAddress64();
  uint16_t       getRxRemoteAddress16();
  const char*         getNodeId();

  AtCommandResponse*  getAtResponse();
  ZBRxResponse*       getRxResponse();
  ZBRxResponse*       getRxData();
  ZBNode              getZBNode();

  bool init(ZBNodeType type, const char* nodeId);    // ZB_COORDINATOR, ZB_GATEWAY, ZB_CLIENT

private:
  int  packetHandle();
  bool setNodeId(const char* id);

  XBee _xbee;
  AtCommandRequest     _atRequest;
  ZBTxRequest          _txRequest;
  AtCommandResponse    _atResp;
  ZBRxResponse         _rxResp;
  ZBNode               _zbNode;
  AtCommandRequest     _atRetryRequest;
  ZBTxRequest          _txRetryRequest;
  ZBRxResponse*        _rxData;
  int                 _returnCode;

  uint8_t _rxDataBuf[MAX_FRAME_DATA_SIZE];

  uint8_t _respWaitStat;  // 0:no wait  1:AtResp   2:TxResp
  uint8_t _sendReqStat;   // 0:no req   1:AtReq    2:TxReq
  bool   _rxDataReady;
  long   _readTimeout;

  void (*_rxCallbackPtr)(ZBRxResponse* data, int* returnCode);
};

#endif  /* ZBEESTACK_H_ */
