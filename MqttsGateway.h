/*
 * MqttsGateway.h
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

#ifndef MQTTSGATEWAY_H_
#define MQTTSGATEWAY_H_

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


#define MQTTS_DEBUG_TOPIC_ID 0x0012

/*=====================================
        Class MqttsGateway for DEBUG
 ======================================*/
class MqttsGateway {
public:
    MqttsGateway();
    ~MqttsGateway();
  #ifdef ARDUINO
        void begin(long baudrate);
  #else
    #ifndef ZBEE_EMULATION
        void begin(char* device, unsigned int bauderate);  /* MBED & LINUX */
    #else
        void begin(long br = 9600);    /* ZBEE_EMULATION */
    #endif
  #endif
    Topics* getTopics();
    XBeeAddress64& getRxRemoteAddress64();
    uint16_t getRxRemoteAddress16();
    void setRetryMax(uint8_t cnt);
    uint8_t getGwId();
    void  setDuration(long msec);
    void setMsgRequestStatus(uint8_t stat);
    void clearMsgRequest();
    uint8_t getMsgRequestType();
    uint8_t getMsgRequestStatus();
    uint8_t getMsgRequestCount();

    void setGwId(MQString* gwId, uint8_t id);
    int  publish(MQString* topic, const char* data, int dataLength);
    int  pubAck(uint16_t topicId, uint16_t msgId, uint8_t rc);
    int  subAck(uint16_t topicId, uint16_t msgId, uint8_t rc, uint8_t flag);
    int  registerTopic(MQString* mqStr, uint16_t topicId);
    int  regAck(uint16_t topicId, uint16_t msgId, uint8_t rc);
    int  gwInfo(uint8_t gwId);
    int  connAck(uint8_t rc);
    int  unsubAck(uint16_t msgId);
    int  advertise(uint16_t duration, uint8_t gwId);
    int  disconnect(uint16_t duration = 0);
    int  pingResp();
    int  willTopicReq();
    int  willMsgReq();
    bool init(const char* gatewayIdName, uint8_t id);
    int  execMsgRequest();
    void recieveMessageHandler(ZBRxResponse* msg, int* returnCode);
    void createTopic(MQString* topic, uint8_t type, TopicCallback callback);
    uint16_t getNextMsgId();
    uint16_t getNextTopicId();
    void setLoopCtrl(uint8_t msgType);
    uint8_t getLoopCtrl();

private:
    int  requestSendMsg(MqttsMessage* msg);
    int  requestPrioritySendMsg(MqttsMessage* mqttsMsgPtr);
    int  broadcast(uint16_t packetReadTimeout);
    int  unicast(uint16_t packetReadTimeout);

    ZBeeStack*       _zbee;
    SerialPort*      _sp;
    Topics           _topics;
    ZBNodeList*      _nodeList;
    SendQue*         _sendQ;
    XTimer           _respTimer;
    XTimer           _advertiseTimer;

    long             _duration;
    MQString*        _gatewayId;
    uint8_t          _gwId;
    uint8_t          _nRetry;
    uint8_t          _nRetryCnt;
    uint16_t         _tRetry;
    uint16_t         _tRetryMsec;
    uint16_t         _msgId;
    uint8_t          _loopCtrl;
    uint16_t         _topicId;
};



#endif /* MQTTSGATEWAY_H_ */
