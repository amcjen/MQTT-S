/*
 * MqttsClient.h
 *
 *               Copyright (c) 2013, tomy-tech.com
 *                       All rights reserved.
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
 * 
 *  Created on: 2013/06/02
 *      Author: Tomoaki YAMAGUCHI
 *     Version:
 *
 */

#ifndef MQTTSCLIENT_H_
#define MQTTSCLIENT_H_

#ifndef ARDUINO
        #include "MQTTS_Defines.h"
#else
        #include <MQTTS_Defines.h>
#endif


#if defined(ARDUINO) && ARDUINO >= 100
        #include "Arduino.h"
        #include <inttypes.h>
        #include <MQTTS.h>
#else
        #if defined(ARDUINO) && ARDUINO < 100
                #include "WProgram.h"
                #include <inttypes.h>
                #include <MQTTS.h>
        #else
                #include <sys/time.h>
                #include <iostream>
                #include "MQTTS.h"
        #endif
#endif

/*=====================================
        Class GatewayHandller
 ======================================*/
class GatewayHandller {
public:
    GatewayHandller();
    bool     isConnected();
    bool     isDisconnected();
    bool     isSearching();
    bool     isFound();
    bool     isLost();
    bool     isInit();
    bool     isPingRequired();
    XBeeAddress64*  getAddress64();
    uint16_t        getAddress16();
    void     setStatus(uint8_t status);
    void     recvGwInfo(MqttsGwInfo* msg);
    void     setLastSendTime();
    void     recvAdvertise(MqttsAdvertise* msg);
    void     setKeepAlive(long msec);
    long  getKeepAlive();
    void     recvPingResp();
private:
    uint8_t        _status;  //  0:init 1:searching 2:find 3:connected  4:disconnect 5:lost
    XBeeAddress64  _addr64;
    uint16_t       _addr16;
    uint8_t        _gwId;
    long       _keepAliveDuration;// PINGREQ interval
    long       _advertiseDuration;

    XTimer          _keepAliveTimer;
    XTimer          _advertiseTimer;
};

/*=====================================
        Class MqttsClient
 ======================================*/
class MqttsClient {
public:
    MqttsClient();
    ~MqttsClient();
  #ifdef ARDUINO
    void begin(long baudrate);
  #else
    void begin(char* device, unsigned int bauderate);  /* MBED & LINUX */

  #endif
    Topics* getTopics();
    void setKeepAlive(long msec);
    void setWillTopic(MQString* topic);
    void setWillMessage(MQString* msg);
    void setQos(uint8_t level);
    void setRetain(bool retain);
    void setClean(bool clean);
    void setRetryMax(uint8_t cnt);
    void setMsgRequestStatus(uint8_t stat);
    void clearMsgRequest();
    uint8_t getMsgRequestType();
    uint8_t getMsgRequestStatus();
    XBeeAddress64& getRxRemoteAddress64();
    uint16_t getRxRemoteAddress16();
    MQString* getClientId();
    uint16_t getNextMsgId();

    //void setClientId(MQString* clientId);
    int  connect();
    int  publish(MQString* topic, const char* data, int dataLength);
    int  registerTopic(MQString* topic);
    int  subscribe(MQString* topic, Callback* callback);
    int  unsubscribe(MQString* topic);
    int  disconnect(uint16_t duration);
    int  willTopic();
    int  willMsg();
    bool init(const char* clientIdName);
    int  execMsgRequest();
    void recieveMessageHandler(ZBRxResponse* msg, int* returnCode);
    void publishHdl(MqttsPublish* msg);
    void createTopic(MQString* topic, uint8_t type, Callback* callback);

private:
    int  searchGw(uint8_t radius);
    int  pingReq(MQString* clietnId);
    int  pingResp();
    int  requestSendMsg(MqttsMessage* msg);
    int  requestPrioritySendMsg(MqttsMessage* mqttsMsgPtr);
    int  broadcast(uint16_t packetReadTimeout);
    int  unicast(uint16_t packetReadTimeout);
    void delayTime(long baseTime);

    ZBeeStack*       _zbee;
    SerialPort*      _sp;
    GatewayHandller  _gwHdl;
    Topics           _topics;
    SendQue*         _sendQ;
    XTimer           _respTimer;
    PublishHandller  _pubHdl;

    uint8_t          _qos;
    uint16_t         _duration;
    MQString*        _clientId;
    uint8_t          _clientFlg;
    uint8_t          _nRetry;
    uint8_t          _nRetryCnt;
    uint16_t         _tRetry;
    uint16_t         _tRetryMsec;
    MQString*         _willTopic;
    MQString*         _willMessage;
    uint16_t         _msgId;
};



#endif /* MQTTSCLIENT_H_ */
