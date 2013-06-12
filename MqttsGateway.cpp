/*
 * MqttsGateway.cpp
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

#ifndef ARDUINO
        #include "MQTTS_Defines.h"
#else
        #include <MQTTS_Defines.h>
#endif


#ifdef ARDUINO
  #include <SoftwareSerial.h>
  #include <MqttsGateway.h>

#endif  /* ARDUINO */

#ifdef MBED
  #include "mbed.h"
  #include "MqttsGateway.h"
#endif  /* MBED */

#ifdef LINUX
  #include "MqttsGateway.h"
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

static MqttsGateway*  theMqttsGw;

/*=======================================================================================

        Class MqttsGateway   for    DEBUG

 =======================================================================================*/
void ResponseHandlerGw(ZBRxResponse* resp, int* returnCode){
        theMqttsGw->recieveMessageHandler(resp, returnCode);
}

MqttsGateway::MqttsGateway(){
    _zbee = new ZBeeStack();
    _sp = new SerialPort();
    _zbee->setSerialPort(_sp);
    _zbee->setRxHandler(ResponseHandlerGw);
    _sendQ = new SendQue();
    _duration = 0;
    _gwId = 0;
    _gatewayId = new MQString();
    _nRetry = 3;
    _tRetryMsec = 10000; // 10 sec
    _nRetryCnt = 0;
    _tRetry = 0;
    //_nodeList = new ZBNodeList(30);
    _msgId = 0;
    _topicId = 0;
    _topics.allocate(MQTTS_MAX_TOPICS);
    _advertiseTimer.start();
    theMqttsGw = this;
}

MqttsGateway::~MqttsGateway(){
  _sendQ->deleteAllRequest();
  delete _zbee;
  delete _sp;
}


#ifdef ARDUINO
void MqttsGateway::begin(long baudrate){
        _sp->begin(baudrate);
}
#endif /* ARDUINO */

#ifdef MBED
void MqttsGateway::begin(long baudrate){
        _sp->begin(baudrate);
}
#endif /* MBED */

#ifdef LINUX
void MqttsGateway::begin(char* device, unsigned int bauderate){
  if( _sp->begin(device, bauderate) < 0){
  fprintf( stdout," Serialport open Error %s", device);
    exit(-1);
  }
}
#endif  /* LINUX */




Topics* MqttsGateway::getTopics(){
    return &_topics;
}


void MqttsGateway::setRetryMax(uint8_t cnt){
    _nRetry = cnt;
}

void MqttsGateway::setDuration(long msec){
    _duration = msec;
}

XBeeAddress64& MqttsGateway::getRxRemoteAddress64(){
    return _zbee->getRxRemoteAddress64();
}

uint16_t MqttsGateway::getRxRemoteAddress16(){
    return _zbee->getRxRemoteAddress16();
}

uint8_t MqttsGateway::getGwId(){
    return _gwId;
}

uint16_t MqttsGateway::getNextMsgId(){
    _msgId++;
    if (_msgId == 0){
        _msgId = 1;
    }
    return _msgId;
}

uint16_t MqttsGateway::getNextTopicId(){
    _topicId++;
    if (_topicId == 0){
        _topicId = 1;
    }
    return _topicId;
}

void MqttsGateway::createTopic(MQString* topic, uint8_t type, TopicCallback callback){
    _topics.addTopic(topic, type);
    _topics.setCallback(topic, callback);
}


void MqttsGateway::clearMsgRequest(){
    _sendQ->deleteRequest(0);
}

uint8_t MqttsGateway::getMsgRequestType(){
    if (_sendQ->getMessage(0)){
        return _sendQ->getMessage(0)->getType();
    }else{
        return 0;
    }
}
uint8_t MqttsGateway::getMsgRequestStatus(){
    return _sendQ->getStatus(0);
}

uint8_t MqttsGateway::getMsgRequestCount(){
  return _sendQ->getCount();
}

void MqttsGateway::setMsgRequestStatus(uint8_t stat){
    _sendQ->setStatus(0,stat);
}

void MqttsGateway::setLoopCtrl(uint8_t loopType){
    _loopCtrl = loopType;
}

uint8_t MqttsGateway::getLoopCtrl(){
    return _loopCtrl;
}

bool MqttsGateway::init(const char* gwNameId, uint8_t id){
    _gwId = id;
    _gatewayId->copy(gwNameId);
    _advertiseTimer.start();
    return _zbee->init(ZB_GATEWAY, gwNameId);
}

/*-------------------------------------------------------------------------*/

int MqttsGateway::requestSendMsg(MqttsMessage* mqttsMsgPtr){
    int index = _sendQ->addRequest((MqttsMessage*)mqttsMsgPtr);
    if ( index >= 0){
        if (_sendQ->getStatus(index) != MQTTS_MSG_RESEND_REQ){
            _sendQ->setStatus(index, MQTTS_MSG_REQUEST);
        }
        return index;
    }
    return MQTTS_ERR_CANNOT_ADD_REQUEST;
}

/*-------------------------------------------------------------------------*/
int MqttsGateway::execMsgRequest(){


    if (_sendQ->getStatus(0) == MQTTS_MSG_REQUEST || _sendQ->getStatus(0) == MQTTS_MSG_RESEND_REQ){
        if (_sendQ->getMessage(0)->getType() == MQTTS_TYPE_GWINFO){
            return broadcast(MQTTS_TIME_RESPONCE);
        }else{
            return unicast(MQTTS_TIME_RESPONCE);
        }
    }else{
        if (_advertiseTimer.isTimeUp(_duration)){
            advertise(_duration, _gwId);
            broadcast(MQTTS_TIME_RESPONCE);
            _advertiseTimer.start();
        }
        _zbee->readPacket();  //  Just read packet, No send Request
    }
    return MQTTS_ERR_NO_ERROR;
}

/*-------------------------------------------------------------------------*/
int MqttsGateway::broadcast(uint16_t packetReadTimeout){
  _zbee->bcastData(_sendQ->getMessage(0)->getMsgBuff()->getBuff(),
                                _sendQ->getMessage(0)->getLength());
  _sendQ->getMessage(0)->setStatus(MQTTS_MSG_COMPLETE);
  _sendQ->deleteRequest(0);
  return MQTTS_ERR_NO_ERROR;
}

int MqttsGateway::unicast(uint16_t packetReadTimeout){
    _zbee->sendData(&getRxRemoteAddress64(), getRxRemoteAddress16(),
                                _sendQ->getMessage(0)->getMsgBuff()->getBuff(),
                                _sendQ->getMessage(0)->getLength(), 0);
    _sendQ->getMessage(0)->setStatus(MQTTS_MSG_COMPLETE);
    _sendQ->deleteRequest(0);
    return MQTTS_ERR_NO_ERROR;
}
/*-------------------------------------------------------------------------*/



/*-----------------------------------------------
 *   Create Messages & send
 ------------------------------------------------*/
/*--------- PUBLISH ------*/
int MqttsGateway::publish(MQString* topic, const char* data, int dataLength){
  uint16_t topicId = _topics.getTopicId(topic);
  if (topicId){
      MqttsPublish mqttsMsg = MqttsPublish();
      mqttsMsg.setTopicId(topicId);
      mqttsMsg.setData((uint8_t*)data, (uint8_t)dataLength);
      mqttsMsg.setMsgId(getNextMsgId());
      mqttsMsg.setFlags(MQTTS_FLAG_QOS_1);
      setLoopCtrl(MQTTS_TYPE_PUBLISH);
      return requestSendMsg((MqttsMessage*)&mqttsMsg);
  }
  return MQTTS_ERR_NO_TOPICID;
}
/*--------- PUBACK ------*/
int MqttsGateway::pubAck(uint16_t topicId, uint16_t msgId, uint8_t rc){
    MqttsPubAck mqttsMsg = MqttsPubAck();
    mqttsMsg.setTopicId(topicId);
    mqttsMsg.setMsgId(msgId);
    mqttsMsg.setReturnCode(rc);
    setLoopCtrl(MQTTS_TYPE_PUBACK);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- SUBACK ------*/
int MqttsGateway::subAck(uint16_t topicId, uint16_t msgId, uint8_t rc, uint8_t flag){
      MqttsSubAck mqttsMsg = MqttsSubAck();
      mqttsMsg.setFlags(flag);
      mqttsMsg.setTopicId(topicId);
      mqttsMsg.setMsgId(msgId);
      mqttsMsg.setReturnCode(rc);
      setLoopCtrl(MQTTS_TYPE_SUBACK);
      return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- REGISTER ------*/
int MqttsGateway::registerTopic(MQString* mqStr, uint16_t topicId){
    MqttsRegister mqttsMsg = MqttsRegister();
    mqttsMsg.setTopicId(topicId);
    mqttsMsg.setMsgId(getNextMsgId());
    mqttsMsg.setTopicName(mqStr);
    setLoopCtrl(MQTTS_TYPE_REGISTER);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- REGACK ------*/
int MqttsGateway::regAck(uint16_t topicId, uint16_t msgId, uint8_t rc){
    MqttsRegAck mqttsMsg = MqttsRegAck();
    mqttsMsg.setTopicId(topicId);
    mqttsMsg.setMsgId(msgId);
    mqttsMsg.setReturnCode(rc);
    setLoopCtrl(MQTTS_TYPE_REGACK);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- DISCONNECT ------*/
int MqttsGateway::disconnect(uint16_t duration){
    MqttsDisconnect mqttsMsg = MqttsDisconnect();
    if (duration){
        mqttsMsg.setDuration(duration);
    }
    setLoopCtrl(MQTTS_TYPE_DISCONNECT);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- GWINFO ------*/
int MqttsGateway::gwInfo(uint8_t gwId){
    MqttsGwInfo mqttsMsg = MqttsGwInfo();
    mqttsMsg.setGwId(gwId);
    setLoopCtrl(MQTTS_TYPE_GWINFO);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- CONNACK ------*/
int MqttsGateway::connAck(uint8_t rc){
    MqttsConnack mqttsMsg = MqttsConnack();
    mqttsMsg.setReturnCode(rc);
    setLoopCtrl(MQTTS_TYPE_CONNACK);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- UNSUBACK ------*/
int MqttsGateway::unsubAck(uint16_t msgId){
    MqttsUnSubAck mqttsMsg = MqttsUnSubAck();
    mqttsMsg.setMsgId(msgId);
    setLoopCtrl(MQTTS_TYPE_UNSUBACK);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- ADVERTISE ------*/
int  MqttsGateway::advertise(uint16_t duration, uint8_t gwId){
    MqttsAdvertise mqttsMsg = MqttsAdvertise();
    mqttsMsg.setDuration(duration);
    mqttsMsg.setGwId(gwId);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- PINGRESP ------*/
int  MqttsGateway::pingResp(){
    MqttsPingResp mqttsMsg = MqttsPingResp();
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- WILLTOPICREQ ------*/
int  MqttsGateway::willTopicReq(){
    MqttsWillTopicReq mqttsMsg = MqttsWillTopicReq();
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- WILLMSGREQ ------*/
int  MqttsGateway::willMsgReq(){
    MqttsWillMsgReq mqttsMsg = MqttsWillMsgReq();
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}



/* ===================================================
                  Receive Message
 =====================================================*/
void MqttsGateway::recieveMessageHandler(ZBRxResponse* recvMsg, int* returnCode){

/*---------  PUBLISH  --------*/
    if (recvMsg->getData()[1] == MQTTS_TYPE_PUBLISH){
       printf("PUBLISH recv\n");
        MqttsPublish mqMsg = MqttsPublish();
        mqMsg.setFrame(recvMsg);
        pubAck(mqMsg.getTopicId(), mqMsg.getMsgId(), MQTTS_RC_ACCEPTED);

/*---------  PUBACK  --------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_PUBACK){
        printf("PUBACK recv\n");

/*---------  SUBSCRIBE  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_SUBSCRIBE){
        printf("SUBSCRIBE recv\n");
        MqttsSubscribe mqMsg = MqttsSubscribe();
        mqMsg.setFrame(recvMsg);
        MQString name;
        name.readBuf(mqMsg.getTopicName());

        MQString* mqStr = name.create();
        uint16_t id = _topics.getTopicId(mqStr);
        if (!id){
            _topics.addTopic(mqStr, mqMsg.getFlags() & 0x3);
            id = getNextTopicId();
            _topics.setTopicId(mqStr, id);
        }
        subAck(id, mqMsg.getMsgId(), MQTTS_RC_ACCEPTED, mqMsg.getFlags());

/*---------  SEARCHGW  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_SEARCHGW){
        printf("SEARCHGW recv\n");
        gwInfo(_gwId);

/*---------  CONNECT  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_CONNECT){
        printf("CONNECT recv\n");
        MqttsConnect mqMsg = MqttsConnect(_gatewayId);
        mqMsg.setFrame(recvMsg->getData(), recvMsg->getData()[0]);
        MQString id;
        id.readBuf(mqMsg.getClientId());

        if (mqMsg.getFlags() && MQTTS_FLAG_WILL){
            willTopicReq();
        }else{
            connAck(MQTTS_RC_ACCEPTED);
        }

/*---------  REGISTER  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_REGISTER){
        printf("REGISTER recv\n");
        MqttsRegister mqMsg = MqttsRegister();
        mqMsg.setFrame(recvMsg);
        regAck(MQTTS_DEBUG_TOPIC_ID, mqMsg.getMsgId(), 0);

/*---------  UNSUBSCRIBE  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_UNSUBSCRIBE){
        printf("UNSUBSCRIBE recv\n");
        MqttsUnsubscribe mqMsg = MqttsUnsubscribe();
        mqMsg.setFrame(recvMsg);
        unsubAck(mqMsg.getMsgId());

/*---------  DISCONNECT  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_DISCONNECT){
        printf("DISCONNECT recv\n");
        MqttsDisconnect mqMsg = MqttsDisconnect();
        memcpy(mqMsg.getMsgBuff()->getBuff(), recvMsg->getData(), recvMsg->getData()[0]);
        //_nodeList->getZBNode(getRxRemoteAddress16())->setNodeStatus(MQTTS_DEVICE_DISCONNECTED);
        disconnect();


/*---------  WILLMSG  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_WILLMSG){
        printf("WILLMSG recv\n");
        MqttsWillMsg mqMsg = MqttsWillMsg();
        // ToDo  willmessage process
        connAck(MQTTS_RC_ACCEPTED);

/*---------  WILLTOPIC  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_WILLTOPIC){
        printf("WILLTOPIC recv\n");
        // ToDo WillTopic process
        willMsgReq();

/*---------  PINGREQ  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_PINGREQ){
        printf("PINGREQ recv\n");
        pingResp();

    }else{
        *returnCode = MQTTS_ERR_NO_ERROR;
    }
}


/*========= End of File ==============*/






