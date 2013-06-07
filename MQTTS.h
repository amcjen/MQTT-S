/*
 * MQTTS.h
 *
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
 *  Created on: 2013/06/8
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 0.3.0
 *
 */

#ifndef MQTTS_H_
#define MQTTS_H_

#ifndef ARDUINO
        #include "MQTTS_Defines.h"
#else
        #include <MQTTS_Defines.h>
#endif


#if defined(ARDUINO) && ARDUINO >= 100
        #include "Arduino.h"
        #include <inttypes.h>
        #include <ZBeeStack.h>
#else
        #if defined(ARDUINO) && ARDUINO < 100
                #include "WProgram.h"
                #include <inttypes.h>
        	#include <ZBeeStack.h>
        #else
                #include <sys/time.h>
                #include <iostream>
                #include "ZBeeStack.h"
        #endif
#endif


#define MQTTS_DEFAULT_KEEPALIVE  20000
#define MQTTS_TIME_SEARCHGW      3000
#define MQTTS_TIME_RESPONCE      3000
#define MQTTS_TIME_RETRY         3000


#define MQTTS_MAX_TOPICS         10
#define MQTTS_MAX_PACKET_LENGTH  60

#define MQTTS_TYPE_ADVERTISE     0x00
#define MQTTS_TYPE_SEARCHGW      0x01
#define MQTTS_TYPE_GWINFO        0x02
#define MQTTS_TYPE_CONNECT       0x04
#define MQTTS_TYPE_CONNACK       0x05
#define MQTTS_TYPE_WILLTOPICREQ  0x06
#define MQTTS_TYPE_WILLTOPIC     0x07
#define MQTTS_TYPE_WILLMSGREQ    0x08
#define MQTTS_TYPE_WILLMSG       0x09
#define MQTTS_TYPE_REGISTER      0x0A
#define MQTTS_TYPE_REGACK        0x0B
#define MQTTS_TYPE_PUBLISH       0x0C
#define MQTTS_TYPE_PUBACK        0x0D
#define MQTTS_TYPE_PUBCOMP       0x0E
#define MQTTS_TYPE_PUBREC        0x0F
#define MQTTS_TYPE_PUBREL        0x10
#define MQTTS_TYPE_SUBSCRIBE     0x12
#define MQTTS_TYPE_SUBACK        0x13
#define MQTTS_TYPE_UNSUBSCRIBE   0x14
#define MQTTS_TYPE_UNSUBACK      0x15
#define MQTTS_TYPE_PINGREQ       0x16
#define MQTTS_TYPE_PINGRESP      0x17
#define MQTTS_TYPE_DISCONNECT    0x18
#define MQTTS_TYPE_WILLTOPICUPD  0x1A
#define MQTTS_TYPE_WILLTOPICRESP 0x1B
#define MQTTS_TYPE_WILLMSGUPD    0x1C
#define MQTTS_TYPE_WILLMSGRESP   0x1D

#define MQTTS_TOPIC_TYPE_NORMAL     0x00
#define MQTTS_TOPIC_TYPE_PREDEFINED 0x01
#define MQTTS_TOPIC_TYPE_SHORT      0x02


#define MQTTS_FLAG_DUP     (0x1 << 7)
#define MQTTS_FLAG_QOS_0   (0x0 << 5)
#define MQTTS_FLAG_QOS_1   (0x1 << 5)
#define MQTTS_FLAG_QOS_2   (0x2 << 5)
#define MQTTS_FLAG_QOS_N1  (0x3 << 5)
#define MQTTS_FLAG_RETAIN  (0x1 << 4)
#define MQTTS_FLAG_WILL    (0x1 << 3)
#define MQTTS_FLAG_CLEAN   (0x1 << 2)

#define MQTTS_PROTOCOL_ID  0x01
#define MQTTS_HEADER_SIZE  2

#define MQTTS_RC_ACCEPTED                  0x00
#define MQTTS_RC_REJECTED_CONGESTION       0x01
#define MQTTS_RC_REJECTED_INVALID_TOPIC_ID 0x02
#define MQTTS_RC_REJECTED_NOT_SUPPORTED    0x03

#define MQTTS_MSG_REQUEST     1
#define MQTTS_MSG_RESEND_REQ  2
#define MQTTS_MSG_WAIT_ACK    3
#define MQTTS_MSG_COMPLETE    4
#define MQTTS_MSG_REJECTED    5


#define MQTTS_GW_INIT         0
#define MQTTS_GW_SEARCHING    1
#define MQTTS_GW_FOUND        2
#define MQTTS_GW_CONNECTED    3
#define MQTTS_GW_DISCONNECTED 4
#define MQTTS_GW_LOST         5

#define MQTTS_ERR_NO_ERROR            0
#define MQTTS_ERR_NOT_CONNECTED      -1
#define MQTTS_ERR_RETRY_OVER         -2
#define MQTTS_ERR_GATEWAY_LOST       -3
#define MQTTS_ERR_CANNOT_ADD_REQUEST -4
#define MQTTS_ERR_NO_TOPICID         -5
#define MQTTS_ERR_REJECTED           -6
#define MQTTS_ERR_WAIT_GWINFO        -7
#define MQTTS_ERR_OUT_OF_MEMORY      -8
#define MQTTS_ERR_PING_REQUIRED      -9
#define MQTTS_ERR_ACK_TIMEOUT       -10
#define MQTTS_ERR_PINGRESP_TIMEOUT  -11

#define MQTTS_TOPIC_MULTI_WILDCARD   '#'
#define MQTTS_TOPIC_SINGLE_WILDCARD  '+'

extern uint16_t getLong(uint8_t* pos);
extern void setLong(uint8_t* pos, uint16_t val);

/*=====================================
        Class MQString
 =====================================*/
class MQString{
public:
    MQString();
    MQString(const char*);
    ~MQString();
    uint8_t getCharLength();
    uint8_t getDataLength();
    int     comp(MQString* str);
    int     comp(const char* str);
    int     ncomp(MQString* str, long n);
    void    copy(MQString* str);
    void    copy(const char* str);
    void    copy(char* str);
    void    writeBuf(uint8_t* buf);
    void    readBuf(uint8_t* buf);
    uint8_t getChar(long index);
    char*  getStr();
    const char* getConstStr();
    bool    isConst();
private:
    void    freeStr();
    uint16_t _length;
    char*    _str;
    const char* _constStr;
};

/*=====================================
        Class MsgBuff
 ======================================*/
class MsgBuff{
public:
    MsgBuff(uint8_t len);
    ~MsgBuff();
    uint8_t* getBuff();
    uint8_t  getLength();
    void copy(MsgBuff* src);
    void copy(uint8_t* data, uint8_t length);
private:
    uint8_t* _buff;
    uint8_t  _length;
};

/*=====================================
        Class MqttsMessage
  =====================================*/
class MqttsMessage {
public:
    MqttsMessage();
    ~MqttsMessage();
    void  setLength(uint8_t length);
    void  setType(uint8_t type);
    bool  setBody(uint8_t* body);
    bool  allocateBody();
    void  setStatus(uint8_t stat);
    uint8_t getLength();
    uint8_t getType();
    uint8_t getStatus();
    uint8_t* getBody();
    uint8_t getBodyLength();
    MsgBuff* getMsgBuff();
    uint8_t getFrameLength();
    bool  copy(MqttsMessage* src);
    void  reset();
    void  setMsgBuff(MsgBuff* buff);
protected:
    MsgBuff* _msgBuff;
private:
    uint8_t  _status; // 1:request 2:sending 3:resending 4:waitingAck  5:complite
    uint8_t  _length;
    uint8_t  _type;
};

/*=====================================
        Class MqttsAdvertize
 ======================================*/
class MqttsAdvertise : public MqttsMessage {
public:
  MqttsAdvertise();
  ~MqttsAdvertise();
  void setGwId(uint8_t id);
  void setDuration(uint16_t duration);
  uint8_t getGwId();
  uint16_t getDuration();

private:
};

/*=====================================
        Class MqttsSearchGw
 ======================================*/
class MqttsSearchGw : public MqttsMessage {
public:
	MqttsSearchGw();
  ~MqttsSearchGw();
  void setRadius(uint8_t radius);
  uint8_t getRadius();

private:
};

/*=====================================
        Class MqttsGwinfo
 ======================================*/
class MqttsGwInfo : public MqttsMessage {
public:
  MqttsGwInfo();
  ~MqttsGwInfo();
  void setGwId(uint8_t id);
  uint8_t getGwId();

private:
};

/*=====================================
         Class MqttsConnect
  ======================================*/
class MqttsConnect : public MqttsMessage {
public:
    MqttsConnect(MQString* id);
    ~MqttsConnect();
    void setFlags(uint8_t flg);
    void setDuration(uint16_t msec);
    void setClientId(MQString* id);
    uint8_t* getClientId();
    uint8_t getFlags();
    uint16_t getDuration();
private:
 };

/*=====================================
        Class MqttsConnack
 ======================================*/
class MqttsConnack : public MqttsMessage  {
public:
    MqttsConnack();
    ~MqttsConnack();
    void setReturnCode(uint8_t rc);
    uint8_t getReturnCode();

private:

};

/*=====================================
         Class MqttsWillTopicReq
  ======================================*/
class MqttsWillTopicReq : public MqttsMessage  {
public:
	MqttsWillTopicReq();
	~MqttsWillTopicReq();

private:

 };

/*=====================================
         Class MqttsWillTopic
  ======================================*/
class MqttsWillTopic : public MqttsMessage  {
public:
	MqttsWillTopic();
	~MqttsWillTopic();
	void setFlags(uint8_t flags);
	void setWillTopic(MQString* topic);
	MQString* getWillTopic();

private:
	uint8_t _flags;
	MQString _ustring;
 };

/*=====================================
         Class MqttsWillMsgReq
  ======================================*/
class MqttsWillMsgReq : public MqttsMessage  {
public:
	MqttsWillMsgReq();
	~MqttsWillMsgReq();

private:

 };

/*=====================================
         Class MqttsWillMsg
  ======================================*/
class MqttsWillMsg : public MqttsMessage  {
public:
	MqttsWillMsg();
	~MqttsWillMsg();
	void setWillMsg(MQString* msg);
	char* getWillMsg();

private:

 };

/*=====================================
         Class MqttsRegister
  ======================================*/
class MqttsRegister : public MqttsMessage  {
public:
	MqttsRegister();
	~MqttsRegister();
	void setTopicId(uint16_t topicId);
	uint16_t getTopicId();
	void setMsgId(uint16_t msgId);
	uint16_t getMsgId();
	void setTopicName(MQString* topicName);
	void setFrame(uint8_t* data, uint8_t len);
	MQString* getTopicName();

private:
	uint16_t _topicId;
	uint16_t _msgId;
	MQString _ustring;

 };

/*=====================================
         Class MqttsRegAck
  ======================================*/
class MqttsRegAck : public MqttsMessage  {
public:
	MqttsRegAck();
	~MqttsRegAck();
	void setTopicId(uint16_t topicId);
	uint16_t getTopicId();
	void setMsgId(uint16_t msgId);
	uint16_t getMsgId();
	void setReturnCode(uint8_t rc);
	uint8_t getReturnCode();

private:

 };

 /*=====================================
         Class MqttsPublish
  ======================================*/
class MqttsPublish : public MqttsMessage  {
public:
	MqttsPublish();
	~MqttsPublish();
	void setFlags(uint8_t flags);
	uint8_t getFlags();
	void setTopicId(uint16_t id);
	uint16_t getTopicId();
	void setMsgId(uint16_t msgId);
	uint16_t getMsgId();
	void setData(uint8_t* data, uint8_t len);
	void setFrame(uint8_t* data, uint8_t len);
	uint8_t* getData();

private:
	uint8_t _flags;
	uint16_t _topicId;
	uint16_t _msgId;
 };

/*=====================================
         Class MqttsPubAck
  ======================================*/
class MqttsPubAck : public MqttsMessage  {
public:
	MqttsPubAck();
	~MqttsPubAck();
	void setTopicId(uint16_t id);
	uint16_t getTopicId();
	void setMsgId(uint16_t msgId);
	uint16_t getMsgId();
	void setReturnCode(uint8_t rc);
	uint8_t getReturnCode();


private:

 };
 /*=====================================
         Class MqttsSubscribe
  ======================================*/
class MqttsSubscribe : public MqttsMessage  {
public:
	MqttsSubscribe();
	~MqttsSubscribe();
	void setFlags(uint8_t flags);
	uint8_t getFlags();
	void setMsgId(uint16_t msgId);
	uint16_t getMsgId();
	void setTopicName(MQString* topicName);
	uint8_t* getTopicName();
	void setTopicId(uint16_t topicId);
	void setFrame(uint8_t* data, uint8_t len);
	uint16_t getTopicId();

private:
	uint16_t _topicId;
	uint8_t  _flags;
	uint16_t _msgId;
 };

/*=====================================
         Class MqttsSubAck
  ======================================*/
class MqttsSubAck : public MqttsMessage  {
public:
	MqttsSubAck();
	~MqttsSubAck();
	void setFlags(uint8_t flags);
	uint8_t getFlags();
	void setMsgId(uint16_t msgId);
	uint16_t getMsgId();
	void setTopicId(uint16_t topicId);
	uint16_t getTopicId();
	void setReturnCode(uint8_t rc);
	uint8_t getReturnCode();

private:

 };

 /*=====================================
         Class MqttsUnsubscribe
  ======================================*/
class MqttsUnsubscribe : public MqttsSubscribe  {
public:
	MqttsUnsubscribe();
	~MqttsUnsubscribe();
	void setFlags(uint8_t flags);
private:

 };

/*=====================================
         Class MqttsUnSubAck
  ======================================*/
class MqttsUnSubAck : public MqttsMessage  {
public:
	MqttsUnSubAck();
	~MqttsUnSubAck();
	void setMsgId(uint16_t msgId);
	uint16_t getMsgId();

private:

 };

/*=====================================
        Class MqttsPingReq
 ======================================*/
class MqttsPingReq : public MqttsMessage  {
public:
	MqttsPingReq(MQString* id);
	~MqttsPingReq();
	//void setClientId(MQString* id);
	char* getClientId();
private:

};

/*=====================================
        Class MqttsPingResp
 ======================================*/
class MqttsPingResp : public MqttsMessage  {
public:
	MqttsPingResp();
	~MqttsPingResp();
private:

};

 /*=====================================
         Class MqttsDisconnect
  ======================================*/
class MqttsDisconnect : public MqttsMessage  {
public:
	MqttsDisconnect();
	~MqttsDisconnect();
	void setDuration(uint16_t duration);
	uint16_t getDuration();
private:

 };



/*=====================================
        Class Callback
 ======================================*/
class Callback {
public:
	Callback();
	void exec(void);
	void exec(ZBRxResponse* data, int* returnCode);
	void exec(MqttsPublish* msg);
};

/*=====================================
        Class Topic
 ======================================*/
class Topic {
public:
    Topic();
    ~Topic();
    uint8_t   getStatus();
    uint16_t  getTopicId();
    MQString*  getTopicName();
    uint8_t   getTopicLength();
    uint8_t   getTopicType();
    Callback* getCallback();
    void     setTopicId(uint16_t id);
    void     setTopicName(MQString* topic);
    void     setStatus(uint8_t stat);
    void     setTopicType(uint8_t type);
    void     execCallback(MqttsPublish* msg);
    void     copy(Topic* src);
    void     setCallback(Callback* callback);
    uint8_t   isWildCard();
    bool     isMatch(Topic* wildCard);
private:
    uint16_t  _topicId;
    //uint8_t   _topicLength;
    uint8_t   _topicType;
    uint8_t   _status;
    MQString*  _topicStr;
    Callback*  _callback;
};

/*=====================================
        Class Topics
 ======================================*/
class Topics {
public:
      Topics();
      ~Topics();
      bool     allocate(uint8_t topicsSize);
      uint16_t  getTopicId(MQString* topic);
      Topic*    getTopic(MQString* topic);
      Topic*    getTopic(uint16_t topicId);
      bool     setTopicId(MQString* topic, uint16_t id);
      bool     setCallback(MQString* topic, Callback* callback);
      bool     setCallback(uint16_t topicId, Callback* callback);
      void     execCallback(uint16_t  topicId, MqttsPublish* msg);
      void     addTopic(MQString* topic, uint8_t type = MQTTS_TOPIC_TYPE_NORMAL);
      Topic*    match(MQString* topic);
      void     setSize(uint8_t size);

private:

    uint8_t   _sizeMax;
    uint8_t   _elmCnt;
    Topic*  _topics;

};

/*=====================================
        Class SendQue  (FIFO)
 ======================================*/
class SendQue {
public:
    SendQue();
    ~SendQue();
    int addRequest(MqttsMessage* msg);
    int addPriorityRequest(MqttsMessage* msg);
    void setStatus(uint8_t index, uint8_t status);
    MqttsMessage* getMessage(uint8_t index);
    uint8_t getStatus(uint8_t index);
    int deleteRequest(uint8_t index);
    void   deleteAllRequest();
    void setQueSize(uint8_t sz);
private:
    uint8_t   _queSize;
    uint8_t   _queCnt;
    MqttsMessage*  _msg[5];
};

/*=====================================
        Class Publish Handler
 ======================================*/
class PublishHandller {
public:
    PublishHandller();
    ~PublishHandller();
    void exec(MqttsPublish* msg, Topics* topics);

};


#endif  /* MQTTS_H_ */
