/*
 * MQTTS.cpp
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
 *  Created on: 2013/05/27
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 0.2.2
 *
 */

#ifdef ARDUINO
  #include <MQTTS_Defines.h>
  #include <MQTTS.h>
#else
  #include "MQTTS_Defines.h"
  #ifndef ZBEE_EMULATION
    #include <termios.h>
  #endif
  #include "MQTTS.h"
  #include <stdio.h>
  #include <sys/time.h>
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <unistd.h>
  #include <stdlib.h>
  #include <string.h>
  #include <fcntl.h>
  #include <errno.h>
#endif
using namespace std;

/*=====================================
        Class Callback
 ======================================*/
Callback::Callback(){

}

void Callback::exec(void){

}

void Callback::exec(MqttsPublish* msg){

}

/*=====================================
        Global functions
 ======================================*/

uint16_t getLong(uint8_t* pos){
  uint16_t val = ((uint16_t)*pos++ << 8);
  return val += *pos;
}

void setLong(uint8_t* pos, uint16_t val){
    *pos++ = (val >> 8) & 0xff;
    *pos = val &0xff;
}
/*=====================================
        Class MqttsMessage
 ======================================*/
MqttsMessage::MqttsMessage(){
    _frameData = NULL;
    _length = 0;
    _status = 0;
    _type = 0;
}
MqttsMessage::~MqttsMessage(){
    if (_frameData != NULL){
        free(_frameData);
    }
}

void MqttsMessage::setType(uint8_t type){
    _type = type;
    if ( _frameData != NULL){
        _frameData[1] = type;
    }
}

void MqttsMessage::setLength(uint8_t length){
    _length = length;
    if ( _frameData != NULL){
        _frameData[0] = length;
    }
}

bool MqttsMessage::setBody(uint8_t* body){
    if (allocateBody()) {
        memcpy(_frameData + MQTTS_HEADER_SIZE, body, _length);
      _frameData[0] = _length;
	    _frameData[1] = _type;
        return true;
    }else{
        return false;
    }
}

bool MqttsMessage::allocateBody(){
    if ( _length ) {
        if (_frameData){
                free(_frameData);
        }
        _frameData = (uint8_t*)calloc(_length, sizeof(uint8_t));
        if ( _frameData){
            _frameData[0] = _length;
            _frameData[1] = _type;
            return true;
        }else{
            return false;
        }
    }else{
        return false;
    }
}

void MqttsMessage::setStatus(uint8_t stat){
    _status = stat;
}

uint8_t MqttsMessage::getLength(){
    return _length;
}

uint8_t MqttsMessage::getType(){
    return _type;
}

uint8_t MqttsMessage::getStatus(){
    return _status;
}

uint8_t* MqttsMessage::getBody(){
    return _frameData + MQTTS_HEADER_SIZE;
}

uint8_t MqttsMessage::getBodyLength(){
    return _length - MQTTS_HEADER_SIZE;
}

uint8_t MqttsMessage::getFrameLength(){
    return _length;
}

uint8_t* MqttsMessage::getFrameData(){
  return _frameData;
}

bool MqttsMessage::copy(MqttsMessage* src){
    setLength(src->getLength());
    setType(src->getType());
    setStatus(src->getStatus());
    _frameData = (uint8_t*)calloc(getLength(), sizeof(uint8_t));
    if (_frameData == NULL){
        return false;
    }
    memcpy(_frameData, src->getFrameData(), src->getLength());
    /////////////////////
    fprintf(stdout, " line 169  MqttsMessage::copy()\n");
    for ( int i = 0; i < src->getLength(); i++){
        fprintf(stdout, " 0x%x", _frameData[i]);
    }
    fprintf(stdout, "\n");
    ////////////////////////
    return true;
}

/*=====================================
        Class MqttsAdvrtise
 ======================================*/
MqttsAdvertise::MqttsAdvertise():MqttsMessage(){
    setLength(5);
    setType(MQTTS_TYPE_ADVERTISE);
    allocateBody();
}

MqttsAdvertise::~MqttsAdvertise(){

}

void MqttsAdvertise::setGwId(uint8_t id){
    getBody()[0] = id;
}

void MqttsAdvertise::MqttsAdvertise::setDuration(uint16_t duration){
    uint8_t* pos = getBody() + 1;
    setLong(pos, duration);
}

uint8_t MqttsAdvertise::getGwId(){
    return getBody()[0];
}

uint16_t MqttsAdvertise::getDuration(){
  uint8_t* pos = getBody() + 1;
    return getLong(pos);
}

/*=====================================
        Class MqttsSearchgw
 ======================================*/
MqttsSearchGw::MqttsSearchGw():MqttsMessage(){
    setLength(3);
    setType(MQTTS_TYPE_SEARCHGW);
    allocateBody();
}

MqttsSearchGw::~MqttsSearchGw(){

}

void MqttsSearchGw::setRadius(uint8_t radius){
  getBody()[0] = radius;
}

uint8_t MqttsSearchGw::getRadius(){
  return getBody()[0];
}

/*=====================================
        Class MqttsGwinfo
 ======================================*/
MqttsGwInfo::MqttsGwInfo(){
  setLength(3);
  setType(MQTTS_TYPE_GWINFO);
  allocateBody();
}

MqttsGwInfo::~MqttsGwInfo(){

}

uint8_t MqttsGwInfo::getGwId(){
    return getBody()[0];
}

void MqttsGwInfo::setGwId(uint8_t id){
    getBody()[0] = id;
}

/*=====================================
         Class MqttsConnect
  ======================================*/
MqttsConnect::MqttsConnect(){
    setLength(ZB_MAX_NODEID + 7);
    setType(MQTTS_TYPE_CONNECT);
    allocateBody();
    setLength(6);
    getBody()[1] = MQTTS_PROTOCOL_ID;

    ///////////////////////////////////////////////
    fprintf(stdout, "\n");
    for ( int i = 0; i < getLength(); i++){
        fprintf(stdout," 0x%x", getFrameData()[i]);
    }
    fprintf(stdout," line 267 MqttsConnect()\n");
    ///////////////////////////////////////////////
}

MqttsConnect::~MqttsConnect(){

}

void MqttsConnect::setFlags(uint8_t flg){
    getBody()[0] = flg & 0x04;
}

uint8_t MqttsConnect::getFlags(){
    return getBody()[0];
}

void MqttsConnect::setDuration(uint16_t msec){
    setLong((uint8_t*)getBody() + 2, msec);
}

uint16_t MqttsConnect::getDuration(){
    return getLong((uint8_t*)getBody() + 2);
}

void MqttsConnect::setClientId(const char* id){
    strcpy((char*)getBody() + 4, id);
    setLength(strlen(id) + 6);
}

/*=====================================
        Class MqttsConnack
 ======================================*/
MqttsConnack::MqttsConnack(){
    setLength(3);
    setType(MQTTS_TYPE_CONNACK);
    allocateBody();
}

MqttsConnack::~MqttsConnack(){

}

void MqttsConnack::setReturnCode(uint8_t rc){
    getBody()[0] = rc;
}

uint8_t MqttsConnack::getReturnCode(){
    return getBody()[0];
}

/*=====================================
       Class MqttsWillTopicReq
======================================*/
MqttsWillTopicReq::MqttsWillTopicReq(){
    setLength(2);
    setType(MQTTS_TYPE_WILLTOPICREQ);
    allocateBody();
}

MqttsWillTopicReq::~MqttsWillTopicReq(){

}

/*=====================================
         Class MqttsWillTopic
  ======================================*/
MqttsWillTopic::MqttsWillTopic(){
    setLength(3);
    setType(MQTTS_TYPE_WILLTOPIC);
    allocateBody();
    _flags = 0;
}

MqttsWillTopic::~MqttsWillTopic(){

}

void MqttsWillTopic::setFlags(uint8_t flags){
    flags &= 0x07;
    if (getFrameData()){
            getBody()[0] = flags;
    }
    _flags = flags;
}

void MqttsWillTopic::setWillTopic(const char* topic){
    setLength(4 + strlen(topic));
    allocateBody();
    setLength(4 + strlen(topic));
    strcpy((char*)getBody() + 1, topic);
    getFrameData()[2] = _flags;
}

char* MqttsWillTopic::getWillTopic(){
    if (getFrameData()){
            return (char*)(getBody() + 1);
    }else{
            return NULL;
    }
}

/*=====================================
         Class MqttsWillMsgReq
  ======================================*/
MqttsWillMsgReq::MqttsWillMsgReq(){
    setLength(2);
    setType(MQTTS_TYPE_WILLMSGREQ);
    allocateBody();

}

MqttsWillMsgReq::~MqttsWillMsgReq(){

}

/*=====================================
         Class MqttsWillMsg
  ======================================*/
MqttsWillMsg::MqttsWillMsg(){
    setLength(2);
    setType(MQTTS_TYPE_WILLMSG);
    allocateBody();
}

MqttsWillMsg::~MqttsWillMsg(){

}

void MqttsWillMsg::setWillMsg(const char* msg){
    setLength(3 + strlen(msg));
    allocateBody();
    setLength(2 + strlen(msg));
    strcpy((char*)getBody(), msg);
}

char* MqttsWillMsg::getWillMsg(){
    if (getFrameData()){
            return (char*)getBody();
    }else{
            return NULL;
    }
}

/*=====================================
         Class MqttsRegister
  ======================================*/
MqttsRegister::MqttsRegister(){
    setLength(6);
    setType(MQTTS_TYPE_REGISTER);
    allocateBody();
    _topicId = 0;
    _msgId = 0;
}

MqttsRegister::~MqttsRegister(){

}
void MqttsRegister::setTopicId(uint16_t topicId){
    if (getFrameData()){
            setLong(getFrameData() + 2, topicId);
    }
    _topicId = topicId;
}
uint16_t MqttsRegister::getTopicId(){
    return _topicId;
}
void MqttsRegister::setMsgId(uint16_t msgId){
    if (getFrameData()){
            setLong(getFrameData() +4, _msgId);
    }
    _msgId = msgId;
}
uint16_t MqttsRegister::getMsgId(){
    return _msgId;

}
void MqttsRegister::setTopicName(const char* topicName){
    setLength(7 + strlen(topicName));
    allocateBody();
    setLength(6 + strlen(topicName));
    strcpy((char*)getBody(), topicName);
    setTopicId(_topicId);
    setMsgId(_msgId);
}
char* MqttsRegister::getTopicName(){
    return (char*)getBody();
}

/*=====================================
         Class MqttsRegAck
  ======================================*/
MqttsRegAck::MqttsRegAck(){
    setLength(7);
    setType(MQTTS_TYPE_REGACK);
    allocateBody();
}
MqttsRegAck::~MqttsRegAck(){

}
void MqttsRegAck::setTopicId(uint16_t topicId){
    setLong((uint8_t*)getBody(), topicId);
}
uint16_t MqttsRegAck::getTopicId(){
    return getLong((unsigned char*)getBody());
}
void MqttsRegAck::setMsgId(uint16_t msgId){
    setLong(getBody()+ 2,msgId);
}
uint16_t MqttsRegAck::getMsgId(){
    return getLong((unsigned char*)getBody()+ 2);
}
void MqttsRegAck::setReturnCode(uint8_t rc){
    getBody()[4] = rc;
}
uint8_t MqttsRegAck::getReturnCode(){
    return (uint8_t)getBody()[4];
}

/*=====================================
         Class MqttsPublish
  ======================================*/
MqttsPublish::MqttsPublish(){
    setLength(5);
    setType(MQTTS_TYPE_PUBLISH);
    allocateBody();
    _topicId = 0;
    _msgId = 0;
    _flags = 0;
}

MqttsPublish::~MqttsPublish(){

}

void MqttsPublish::setFlags(uint8_t flags){
    _flags = flags & 0xf3;
    if (getFrameData()){
            getBody()[0] = _flags;
    }
}

uint8_t MqttsPublish::getFlags(){
    return _flags;
}

void MqttsPublish::setTopicId(uint16_t id){
  if (getFrameData()){
     setLong((uint8_t*)(getBody() + 1), id);
  }
  _topicId = id;
}

uint16_t MqttsPublish::getTopicId(){
    return _topicId;
}
void MqttsPublish::setMsgId(uint16_t msgId){
    if (getFrameData()){
       setLong((uint8_t*)(getBody() + 3), msgId);
    }
    _msgId = msgId;
}

uint16_t MqttsPublish::getMsgId(){
    return _msgId;
}
void MqttsPublish::setData(uint8_t* data, uint8_t len){
    setLength(7 + len);
    allocateBody();
    memcpy(getBody() + 5, data, len);
    setTopicId(_topicId);
    setMsgId(_msgId);
    setFlags(_flags);
}
uint8_t*  MqttsPublish::getData(){
    return (uint8_t*)(getBody() + 6);
}

/*=====================================
         Class MqttsPubAck
 ======================================*/
MqttsPubAck::MqttsPubAck(){
    setLength(7);
    setType(MQTTS_TYPE_PUBACK);
    allocateBody();
}
MqttsPubAck::~MqttsPubAck(){

}
void MqttsPubAck::setTopicId(uint16_t topicId){
    setLong((uint8_t*)getBody(), topicId);
}
uint16_t MqttsPubAck::getTopicId(){
    return getLong((unsigned char*)getBody());
}
void MqttsPubAck::setMsgId(uint16_t msgId){
    setLong(getBody()+ 2,msgId);
}
uint16_t MqttsPubAck::getMsgId(){
    return getLong((unsigned char*)getBody()+ 2);
}
void MqttsPubAck::setReturnCode(uint8_t rc){
    getBody()[4] = rc;
}
uint8_t MqttsPubAck::getReturnCode(){
    return (uint8_t)getBody()[4];
}

 /*=====================================
         Class MqttsSubscribe
  ======================================*/
MqttsSubscribe::MqttsSubscribe(){
    setLength(7);
    setType(MQTTS_TYPE_SUBSCRIBE);
    allocateBody();
    _topicId = 0;
    _msgId = 0;
    _flags = 0;
}

MqttsSubscribe::~MqttsSubscribe(){

}

void MqttsSubscribe::setFlags(uint8_t flags){
    _flags = flags  & 0xe3;
    if (getFrameData()){
              getBody()[0] = flags;
      }
}

uint8_t MqttsSubscribe::getFlags(){
    return _flags;
}

void MqttsSubscribe::setTopicId(uint16_t id){
    _topicId = id;
    if (getFrameData()){
       setLong((uint8_t*)(getBody() + 3), id);
    }
}

uint16_t MqttsSubscribe::getTopicId(){
    return _topicId;
}
void MqttsSubscribe::setMsgId(uint16_t msgId){
    _msgId = msgId;
    if (getFrameData()){
       setLong((uint8_t*)(getBody() + 1), msgId);
    }
}

uint16_t MqttsSubscribe::getMsgId(){
    return _msgId;
}
void MqttsSubscribe::setTopicName(char* data){
    setLength(8 + strlen(data));
    allocateBody();
    strcpy((char*)getBody(), (const char*)data);
    setLength(7 + strlen(data));
    setMsgId(_msgId);
    setFlags(_flags);
}
uint8_t*  MqttsSubscribe::getTopicName(){
    return (uint8_t*)(getBody() + 6);
}

/*=====================================
         Class MqttsSubAck
  ======================================*/
MqttsSubAck::MqttsSubAck(){
    setLength(8);
    setType(MQTTS_TYPE_SUBACK);
    allocateBody();
}

MqttsSubAck::~MqttsSubAck(){

}

void MqttsSubAck::setFlags(uint8_t flags){
    getBody()[0] = flags & 0x60;
}

uint8_t MqttsSubAck::getFlags(){
    return getBody()[0];
}

void MqttsSubAck::setTopicId(uint16_t id){
    setLong((uint8_t*)(getBody() + 1), id);
}

uint16_t MqttsSubAck::getTopicId(){
    return getLong(getBody() + 1);
}
void MqttsSubAck::setMsgId(uint16_t msgId){
   setLong((uint8_t*)(getBody() + 3), msgId);
}

uint16_t MqttsSubAck::getMsgId(){
    return getLong(getBody() + 3);
}
void MqttsSubAck::setReturnCode(uint8_t rc){
    getBody()[6] = rc;
}
uint8_t  MqttsSubAck::getReturnCode(){
    return getBody()[5];
}


 /*=====================================
         Class MqttsUnsubscribe
  ======================================*/
MqttsUnsubscribe::MqttsUnsubscribe(){
    setType(MQTTS_TYPE_UNSUBSCRIBE);
}
MqttsUnsubscribe::~MqttsUnsubscribe(){

}
void MqttsUnsubscribe::setFlags(uint8_t flags){
    if (getFrameData()){
              getBody()[0] = flags & 0x02;
    }
}
/*=====================================
         Class MqttsUnSubAck
  ======================================*/
MqttsUnSubAck::MqttsUnSubAck(){
    setLength(2);
    setType(MQTTS_TYPE_UNSUBACK);
    allocateBody();
}

MqttsUnSubAck::~MqttsUnSubAck(){

}

void MqttsUnSubAck::setMsgId(uint16_t msgId){
    setLong((uint8_t*)(getBody() + 0), msgId);
}

uint16_t MqttsUnSubAck::getMsgId(){
    return getLong(getBody());
}

/*=====================================
        Class MqttsPingReq
 ======================================*/
MqttsPingReq::MqttsPingReq(){
  setLength(ZB_MAX_NODEID + 3);
  setType(MQTTS_TYPE_PINGREQ);
  allocateBody();
  setLength(2);
}
MqttsPingReq::~MqttsPingReq(){

}
void MqttsPingReq::setClientId(const char* id){
    setLength(strlen(id) + 2);
    strcpy((char*)getBody(), id);
}
char* MqttsPingReq::getClientId(){
    return (char*)getBody();
}

/*=====================================
        Class MqttsPingResp
 ======================================*/
MqttsPingResp::MqttsPingResp(){
    setLength(2);
    setType(MQTTS_TYPE_PINGRESP);
}
MqttsPingResp::~MqttsPingResp(){

}

 /*=====================================
         Class MqttsDisconnect
  ======================================*/
MqttsDisconnect::MqttsDisconnect(){
    setLength(4);
    setType(MQTTS_TYPE_DISCONNECT);
}
MqttsDisconnect::~MqttsDisconnect(){

}
void MqttsDisconnect::setDuration(uint16_t duration){
    setLong((uint8_t*)getBody(), duration);
}
uint16_t MqttsDisconnect::getDuration(){
    return getLong((uint8_t*)getBody());
}


/*=====================================
        Class Topic
 ======================================*/
Topic::Topic(){
    _topicStr = NULL;
    _callback = NULL;
    _topicLength = 0;
    _topicId = 0;
    _status = 0;
    _topicType = 0;
}

Topic::~Topic(){
    if (_topicStr != NULL){
        free(_topicStr);
    }
}

uint8_t Topic::getStatus(){
    return _status;
}

uint16_t Topic::getTopicId(){
    return _topicId;
}

uint8_t* Topic::getTopicName(){
    return _topicStr;
}

uint8_t Topic::getTopicLength(){
    return _topicLength;
}

uint8_t Topic::getTopicType(){
    return _topicType;
}

Callback* Topic::getCallback(){
    return _callback;
}

void Topic::setTopicId(uint16_t id){
    _topicId = id;
}

void Topic::setStatus(uint8_t stat){
    _topicId = stat;
}

void Topic::setTopicType(uint8_t type){
    _topicType = type;
}

void Topic::setTopicLength(uint8_t len){
    _topicLength = len;
}

bool Topic::setTopicName(char* topic){
    _topicLength = 0;
    if (topic){
        _topicStr = (uint8_t*)calloc(strlen((char*)topic) + 1, sizeof(char));
        if ( _topicStr == 0 ){
            return false;
        }else{
            strcpy((char*)_topicStr, (const char*)topic);
            _topicLength = strlen((char*)topic);
        }
    }
    return true;
}

void Topic::setCallback(Callback* callback){
    _callback = callback;
}

void Topic::execCallback(MqttsPublish* msg){
    if(_callback != NULL){
        _callback->exec(msg);
    }
}

void Topic::copy(Topic*src){
    setTopicId(src->getTopicId());
    setStatus(src->getStatus());
    setTopicLength(src->getTopicLength());
    setTopicType(src->getTopicType());
    setCallback(src->getCallback());
    setCallback(_callback);
    setTopicName((char*)src->getTopicName());
}

bool Topic::isWildCard(){
    return (getTopicName()[strlen((const char*) getTopicName()) - 1] == '+') ||
            (getTopicName()[strlen((const char*) getTopicName()) - 1] == '#');
}

bool Topic::isMatch(Topic* wildCard){
    uint8_t pos = strlen((char*)wildCard->getTopicName()) - 1;
    if ( (char)wildCard->getTopicName()[pos] == '+' &&
          strncmp((char*)wildCard->getTopicName(), (char*)getTopicName(), pos -1) == 0){
        for ( ; pos < strlen((char*)getTopicName()); pos++){
            if ((char)getTopicName()[pos] == '/'){
                return false;
            }
        }
        return true;
    }else if((char)wildCard->getTopicName()[pos] == '#' &&
               strncmp((char*)wildCard->getTopicName(), (char*)getTopicName(), pos -1) == 0 &&
               (char)getTopicName()[pos] != '#'){
        return true;
    }
    return false;
}

/*=====================================
        Class Topics
 ======================================*/
Topics::Topics(){
	_sizeMax = 0;
	_elmCnt = 0;
	_topics = NULL;
}

Topics::~Topics() {
    if (_topics != 0){
        for ( int i = 0; i < _elmCnt; i++){
            free(_topics[i].getTopicName());
        }
        free(_topics);
    }
}

bool Topics::allocate(uint8_t size){
    _elmCnt = 0;
      _topics = (Topic*)calloc(size, sizeof(Topic));
    if (_topics == 0){
        _sizeMax = 0;
        return false;
    }else{
        _sizeMax = size;
        return true;
    }
}


uint16_t Topics::getTopicId(char* topic){
    Topic *p = getTopic(topic);
    if ( p != NULL) {
        return p->getTopicId();
    }
    return 0;
}

Topic* Topics::getTopic(char* topic) {
    for (int i = 0; i < _elmCnt; i++) {
        if ( (strcmp((const char*)_topics[i].getTopicName(), (const char*)topic) == 0)) {
            return &_topics[i];
        }
    }
      return NULL;
}

Topic* Topics::getTopic(uint16_t id) {
    for (int i = 0; i < _elmCnt; i++) {
        if ( _topics[i].getTopicId() == id) {
            return &_topics[i];
        }
    }
      return NULL;
}

bool Topics::setTopicId(char* topic, uint16_t id){
    Topic* p = getTopic(topic);
    if ( p != NULL) {
        p->setTopicId(id);
        return true;
    }else{
        return false;
    }
}

bool Topics::setCallback(char* topic, Callback* callback){
    Topic* p = getTopic(topic);
    if ( p != NULL) {
        p->setCallback(callback);
        return true;
    }else{
        return false;
    }
}
bool Topics::setCallback(uint16_t topicId, Callback* callback){
    Topic* p = getTopic(topicId);
    if ( p != NULL) {
        p->setCallback(callback);
        return true;
    }else{
        return false;
    }
}
void Topics::execCallback(uint16_t topicId, MqttsPublish* msg){
  Topic* p = getTopic(topicId);
      if ( p != NULL) {
          p->execCallback(msg);
      }
}

void Topics::addTopic(char* topic, uint8_t type){
    if ( _elmCnt < _sizeMax && getTopic(topic) == NULL){
        Topic* saveTopics = _topics;
        Topic* newTopics = (Topic*)calloc(_elmCnt + 1, sizeof(Topic));
        if (newTopics != NULL){
            _topics = newTopics;
            for(int i = 0; i < _elmCnt; i++){
                _topics[i].copy(&saveTopics[i]);
                saveTopics[i].setTopicName((char*)NULL);
            }

            _topics[_elmCnt].setTopicName(topic);
            _topics[_elmCnt].setTopicType(type);
            _elmCnt++;
            if (saveTopics){
                delete saveTopics;
            }
        }
    }
}

Topic* Topics::match(char* topic){
    for ( int i = 0; i< _elmCnt; i++){
        if (_topics[i].isWildCard()){
            if (getTopic(topic)->isMatch(&_topics[i])){
               return &_topics[i];
            }
        }
    }
    return NULL;
}

void Topics::setSize(uint8_t size){
    _sizeMax = size;
}

/*=====================================
        Class GatewayHandller
 ======================================*/
GatewayHandller::GatewayHandller(){
    _gwId = 0;
	_status = 0;
	_keepAliveDuration = 0;
	_advertiseDuration = 0;
	_addr16 = 0;
}

bool GatewayHandller::isConnected(){
    if (_status == MQTTS_GW_CONNECTED){
        return true;
    }else{
        return false;
    }
}

bool GatewayHandller::isDisconnected(){
    if (_status == MQTTS_GW_DISCONNECTED){
        return true;
    }else{
        return false;
    }
}

bool GatewayHandller::isSearching(){
    if (_status == MQTTS_GW_SEARCHING){
        return true;
    }else{
        return false;
    }
}

bool GatewayHandller::isFound(){
    if (_status == MQTTS_GW_FOUND){
        return true;
    }else{
        return false;
    }
}

bool GatewayHandller::isLost(){
    if ( _advertiseTimer.isTimeUp(_advertiseDuration)){
        _status = MQTTS_GW_LOST;
        return true;
    }else{
        return false;
    }
}

bool GatewayHandller::isInit(){
    if (_status == MQTTS_GW_INIT){
        return true;
    }else{
        return false;
    }
}

bool GatewayHandller::isPingRequired(){
    return _keepAliveTimer.isTimeUp(_keepAliveDuration);
}

XBeeAddress64* GatewayHandller::getAddress64(){
    return &_addr64;
}

uint16_t GatewayHandller::getAddress16(){
    return _addr16;
}

void GatewayHandller::setStatus(uint8_t status){
    _status = status;
}

void GatewayHandller::recvGwInfo(MqttsGwInfo* msg){
    if (_status == MQTTS_GW_LOST || _status == MQTTS_GW_INIT || _status == MQTTS_GW_SEARCHING){
        if (msg->getLength() == 3){
            _addr64 = theMqtts->getRxRemoteAddress64();
            _addr16 = theMqtts->getRxRemoteAddress16();
            _gwId = msg->getBody()[0];
            _status = MQTTS_GW_FOUND;
        }
    }
}

void GatewayHandller::recvPingResp(){
    _keepAliveTimer.start();
}

void GatewayHandller::setLastSendTime(){
    _keepAliveTimer.start();
}

void GatewayHandller::recvAdvertise(MqttsAdvertise* adv){
    if ( adv->getGwId() == _gwId){
        _advertiseTimer.start();
        _advertiseDuration = adv->getDuration();
        _gwId = adv->getGwId();
    }
}

void GatewayHandller::setKeepAlive(long msec){
	_keepAliveDuration = msec;
}

uint16_t GatewayHandller::getKeepAlive(){
        return _keepAliveDuration;
}

/*=====================================
        Class SendQue
 ======================================*/
SendQue::SendQue(){
    _msg = 0;
    _queCnt = 0;
    _queSize = 5;
}
SendQue::~SendQue(){
    if (_msg != NULL){
        for( int i = 0; i < _queCnt; i++){
            delete &_msg[i];
        }
    }
}

uint8_t SendQue::addRequest(MqttsMessage* msg){
    if ( _queCnt < _queSize){
        if ( _queCnt > 0 &&_msg[0].getStatus() == MQTTS_MSG_COMPLETE){
            deleteRequest(0);
        }
        MqttsMessage* saveMsg = _msg;
        MqttsMessage* newMsg = (MqttsMessage*)calloc(_queCnt + 1, sizeof(MqttsMessage));
        if (newMsg != NULL){
            _msg = newMsg;
            for(int i = 0; i < _queCnt; i++){
                _msg[i].copy(&saveMsg[i]);
            }
            _msg[_queCnt].copy(msg);
            _queCnt++;
            return _queCnt -1;
        }else{
            return -1; // can't allocate a que element
        }
    }
    return -2; // Over Que size
}

uint8_t SendQue::addPriorityRequest(MqttsMessage* msg){
    if ( _queCnt < _queSize){
        if ( _msg[0].getStatus() == MQTTS_MSG_COMPLETE){
            deleteRequest(0);
        }
        MqttsMessage* saveMsg = _msg;
        MqttsMessage* newMsg = (MqttsMessage*)calloc(_queCnt + 1, sizeof(MqttsMessage));
        if (newMsg != NULL){
            _msg = newMsg;
            _queCnt++;
            for(int i = 1; i < _queCnt; i++){
                _msg[i].copy(&saveMsg[i - 1]);
            }
            _msg[0].copy(msg);
        }else{
            return -1;
        }
        return 0;
    }
    return -2;
}

void SendQue::setStatus(uint8_t index, uint8_t status){
    if ( index < _queCnt){
        _msg[index].setStatus(status);
    }
}

void SendQue::setQueSize(uint8_t sz){
  _queSize = sz;
}

MqttsMessage* SendQue::getMessage(uint8_t index){
  if ( index < _queCnt){
      return &_msg[index];
  }
  return NULL;
}

uint8_t SendQue::getStatus(uint8_t index){
  if ( index < _queCnt){
      return _msg[index].getStatus();
  }
  return -1;
}

uint8_t SendQue::deleteRequest(uint8_t index){
    if ( index < _queCnt){
          MqttsMessage* saveMsg = _msg;
          MqttsMessage* newMsg = (MqttsMessage*)calloc(_queCnt - 1, sizeof(MqttsMessage));
          if (newMsg != NULL){
              _msg = newMsg;
              _queCnt--;
              for(int i = 0; i < index; i++){
                  _msg[i].copy(&saveMsg[i]);
              }
              delete &_msg[index];
              for(int i = index; i < _queCnt; i++){
                  _msg[i].copy(&saveMsg[i+1]);
              }
              return 0;

          }else{
              return -1;
          }
          return 0;
      }
    return -2;
}

void   SendQue::deleteAllRequest(){
    while ( _queCnt > 0){
        deleteRequest(0);
    }
}
/*=====================================
        Class MqttsClient
 ======================================*/
void ResponseHandler(ZBRxResponse* resp, int* returnCode){
	theMqtts->recieveMessageHandler(resp, returnCode);
}

MqttsClient::MqttsClient(){
	theMqtts = this;
    _zbee.setSerialPort(&_sp);
    _zbee.setRxHandler(ResponseHandler);
    _qos = 0;
    _duration = 0;
    _clientId = NULL;
    _clientFlg = 0;
    _nRetry = 3;
    _tRetryMsec = 10000; // 10 sec
    _nRetryCnt = 0;
    _tRetry = 0;
    _willTopic = _willMessage = " ";
    _gwHdl.setKeepAlive(MQTTS_DEFAULT_KEEPALIVE);
    _msgId = 0;
    _tSearch = MQTTS_TIME_SEARCH;
    _topics.setSize(MQTTS_MAX_TOPICS);
}

MqttsClient::~MqttsClient(){
  _sendQ.deleteAllRequest();
}

#ifdef ARDUINO
void MqttsClient::begin(long baudrate){
    _sp.begin(baudrate);
}
#else
#ifndef ZBEE_EMULATION
	void MqttsClient::begin(char* device, unsigned int bauderate){
		if( _sp.begin(device, bauderate) < 0){
		fprintf( stdout," Serialport open Error %s", device);
		  exit(-1);
		}
	}
#else
	void MqttsClient::begin(){

	}
#endif     /* ZBEE_EMULATION */
#endif     /* ARDUINO */

Topics* MqttsClient::getTopics(){
    return &_topics;
}

void MqttsClient::setKeepAlive(long msec){
    _gwHdl.setKeepAlive(msec);
}

void MqttsClient::setWillTopic(const char* topic){
    _willTopic = topic;
    _clientFlg |= MQTTS_FLAG_WILL;
}

void MqttsClient::setWillMessage(const char* msg){
    _willMessage = msg;
    _clientFlg |= MQTTS_FLAG_WILL;
}

void MqttsClient::setQos(uint8_t level){
    if (level == 0){
            _clientFlg |= MQTTS_FLAG_QOS_0;
    }else if (level == 1){
            _clientFlg |=MQTTS_FLAG_QOS_1;
    }
    _qos = level;
}

void MqttsClient::setRetain(bool retain){
    _clientFlg |= MQTTS_FLAG_RETAIN;
}

void MqttsClient::setClean(bool clean){
    _clientFlg |= MQTTS_FLAG_CLEAN;
}

void MqttsClient::setRetryMax(uint8_t cnt){
    _nRetry = cnt;
}

XBeeAddress64& MqttsClient::getRxRemoteAddress64(){
    return _zbee.getRxRemoteAddress64();
}

uint16_t MqttsClient::getRxRemoteAddress16(){
    return _zbee.getRxRemoteAddress16();
}

void MqttsClient::setClientId(const char* clientId){
    _clientId = clientId;
    _zbee.setNodeId(clientId);
}

const char* MqttsClient::getClientId(){
    return _clientId;
}

uint16_t MqttsClient::getNextMsgId(){
    _msgId++;
    if (_msgId == 0){
        _msgId = 1;
    }
    return _msgId;
}

void MqttsClient::createTopic(char* topic, uint8_t type, Callback* callback){
    _topics.addTopic(topic, type);
    _topics.setCallback(topic, callback);
}

/*=====================================
        Class Publish Handler
 ======================================*/
PublishHandller::PublishHandller(){

}

PublishHandller::~PublishHandller(){

}

void PublishHandller::exec(MqttsPublish* msg, Topics* topics){
    if (topics->getTopic(msg->getTopicId())){
        topics->getTopic(msg->getTopicId())->execCallback(msg);
    }else{
        return;
    }
}

/* ===================================================
                  Receive Message
 =====================================================*/
void MqttsClient::recieveMessageHandler(ZBRxResponse* recvMsg, int* returnCode){
    if ( _gwHdl.isSearching() && (recvMsg->getData()[1] != MQTTS_TYPE_GWINFO)){
        *returnCode = MQTTS_ERR_NO_ERROR;

/*---------  PUBLISH  --------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_PUBLISH){
        MqttsPublish mqMsg = MqttsPublish();
        memcpy((mqMsg.getFrameData()), recvMsg->getData(), mqMsg.getFrameLength());
        _pubHdl.exec(&mqMsg,&_topics);

/*---------  PUBACK  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_PUBACK && _sendQ.getStatus(0) == MQTTS_MSG_WAIT_ACK){
        MqttsPubAck mqMsg = MqttsPubAck();
        memcpy((mqMsg.getFrameData()), recvMsg->getData(), mqMsg.getFrameLength());

        fprintf(stdout,"\nPUBACK ReturnCode=%d\n", mqMsg.getReturnCode()); //////////////
        if (mqMsg.getMsgId() == getLong(_sendQ.getMessage(0)->getBody() + 2)){
          if (mqMsg.getReturnCode() == MQTTS_RC_REJECTED_CONGESTION){
              if (_nRetryCnt++ < _nRetry){
                  _sendQ.getMessage(0)->setStatus(MQTTS_MSG_REQUEST);
              }else{
                  *returnCode = MQTTS_ERR_REJECTED;
              }
          }else if (mqMsg.getReturnCode() == MQTTS_RC_ACCEPTED){
              _sendQ.getMessage(0)->setStatus(MQTTS_MSG_COMPLETE);
              *returnCode = MQTTS_ERR_NO_ERROR;
          }else{
              *returnCode = MQTTS_ERR_REJECTED;
          }
        }else{
            *returnCode = MQTTS_ERR_NO_ERROR;
        }

/*---------  PINGRESP  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_PINGRESP){
        _gwHdl.recvPingResp();

/*---------  ADVERTISE  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_ADVERTISE){
        MqttsAdvertise mqMsg = MqttsAdvertise();
        memcpy((mqMsg.getFrameData()), recvMsg->getData(), mqMsg.getFrameLength());
        _gwHdl.recvAdvertise(&mqMsg);
        fprintf(stdout," ADVERTISE recived\n");
        *returnCode = MQTTS_ERR_NO_ERROR;

/*---------  GWINFO  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_GWINFO){
        MqttsGwInfo mqMsg = MqttsGwInfo();
        memcpy((mqMsg.getFrameData()), recvMsg->getData(), mqMsg.getFrameLength());
        _gwHdl.recvGwInfo(&mqMsg);
        fprintf(stdout," GWINFO recived\n");
        *returnCode = MQTTS_ERR_NO_ERROR;

/*---------  CANNACK  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_CONNACK){
        if (_qos == 1 && _sendQ.getStatus(0) == MQTTS_MSG_WAIT_ACK &&
             _sendQ.getMessage(0)->getType() == MQTTS_TYPE_CONNECT){
           MqttsConnack mqMsg = MqttsConnack();
           memcpy((mqMsg.getFrameData()), recvMsg->getData(), mqMsg.getFrameLength());
           if (mqMsg.getReturnCode() == MQTTS_RC_REJECTED_CONGESTION){
               if  (_sendQ.getMessage(0)->getType() == MQTTS_TYPE_WILLTOPIC ||
                    _sendQ.getMessage(0)->getType() == MQTTS_TYPE_WILLMSG){
                   _sendQ.deleteRequest(0);
               }
               if (_nRetryCnt++ < _nRetry){
                   _sendQ.getMessage(0)->setStatus(MQTTS_MSG_REQUEST);
               }else{
                   *returnCode = MQTTS_ERR_REJECTED;
               }
           }else if (mqMsg.getReturnCode() == MQTTS_RC_ACCEPTED){
               if  (_sendQ.getMessage(0)->getType() == MQTTS_TYPE_WILLTOPIC){
                   _sendQ.deleteRequest(0);
               }
               _sendQ.getMessage(0)->setStatus(MQTTS_MSG_COMPLETE);
               _gwHdl.setStatus(MQTTS_GW_CONNECTED);
               *returnCode = MQTTS_ERR_NO_ERROR;
           }else{
               *returnCode = MQTTS_ERR_REJECTED;
           }
        }
        fprintf(stdout," CANNACK recived\n");

/*---------  REGISTER  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_REGISTER){
        MqttsRegister mqMsg = MqttsRegister();
        memcpy((mqMsg.getFrameData()), recvMsg->getData(), mqMsg.getFrameLength());
        if (_topics.match(mqMsg.getTopicName())){
            _topics.addTopic(mqMsg.getTopicName());
            _topics.setTopicId(mqMsg.getTopicName(),mqMsg.getTopicId());
            _topics.setCallback(mqMsg.getTopicId(),_topics.match(mqMsg.getTopicName())->getCallback());
        }
        *returnCode = MQTTS_ERR_NO_ERROR;

                fprintf(stdout," REGISTER recived\n");
/*---------  REGACK  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_REGACK){
        if (_sendQ.getStatus(0) == MQTTS_MSG_WAIT_ACK &&
            _sendQ.getMessage(0)->getType() == MQTTS_TYPE_REGISTER){
            MqttsRegAck mqMsg = MqttsRegAck();
            memcpy((mqMsg.getFrameData()), recvMsg->getData(), mqMsg.getFrameLength());
            if (mqMsg.getMsgId() == getLong(_sendQ.getMessage(0)->getBody() + 2)){
                if (getLong((uint8_t*)_sendQ.getMessage(0)->getBody()+4)){
                    if (mqMsg.getReturnCode() == MQTTS_RC_REJECTED_CONGESTION){
                       if (_nRetryCnt++ < _nRetry){
                           delayTime();
                           _sendQ.getMessage(0)->setStatus(MQTTS_MSG_REQUEST);
                       }else{
                           *returnCode = MQTTS_ERR_REJECTED;
                       }
                    }else if (mqMsg.getReturnCode() == MQTTS_RC_ACCEPTED){
                        _topics.setTopicId((char*)(_sendQ.getMessage(0)->getBody() + 4), mqMsg.getTopicId());
                        *returnCode = MQTTS_ERR_NO_ERROR;
                    }else{
                        *returnCode = MQTTS_ERR_REJECTED;
                    }
                    _sendQ.getMessage(0)->setStatus(MQTTS_MSG_COMPLETE);
                }else{
                    *returnCode = MQTTS_ERR_NO_ERROR;
                }
            }else{
                *returnCode = MQTTS_ERR_NO_ERROR;
            }
        }

/*---------  SUBACK  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_SUBACK && _sendQ.getStatus(0) == MQTTS_MSG_WAIT_ACK){
        MqttsSubAck mqMsg = MqttsSubAck();
        memcpy((mqMsg.getFrameData()), recvMsg->getData(), mqMsg.getFrameLength());

        fprintf(stdout,"\nSUBACK ReturnCode=%d\n", mqMsg.getReturnCode()); //////////////
        if (mqMsg.getMsgId() == getLong(_sendQ.getMessage(0)->getBody() + 1)){
          if (mqMsg.getReturnCode() == MQTTS_RC_REJECTED_CONGESTION){
              if (_nRetryCnt++ < _nRetry){
                  _sendQ.getMessage(0)->setStatus(MQTTS_MSG_REQUEST);
              }else{
                  *returnCode = MQTTS_ERR_REJECTED;
              }
          }else if (mqMsg.getReturnCode() == MQTTS_RC_ACCEPTED){
              _sendQ.getMessage(0)->setStatus(MQTTS_MSG_COMPLETE);
              if (_sendQ.getMessage(0)->getBodyLength() > 5){ // TopicName
                  _topics.setTopicId((char*)(_sendQ.getMessage(0)->getBody() + 4), mqMsg.getTopicId());
              }
              *returnCode = MQTTS_ERR_NO_ERROR;

          }else{
              *returnCode = MQTTS_ERR_REJECTED;
          }
        }else{
            *returnCode = MQTTS_ERR_NO_ERROR;
        }

/*---------  UNSUBACK  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_UNSUBACK && _sendQ.getStatus(0) == MQTTS_MSG_WAIT_ACK){
        MqttsSubAck mqMsg = MqttsSubAck();
        memcpy((mqMsg.getFrameData()), recvMsg->getData(), mqMsg.getFrameLength());

        if (mqMsg.getMsgId() == getLong(_sendQ.getMessage(0)->getBody() + 1)){
              _sendQ.getMessage(0)->setStatus(MQTTS_MSG_COMPLETE);
        }
        *returnCode = MQTTS_ERR_NO_ERROR;

/*---------  WILLMSGRESP  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_WILLMSGRESP && _sendQ.getStatus(0) == MQTTS_MSG_WAIT_ACK){
        *returnCode = MQTTS_ERR_NO_ERROR;

/*---------  WILLTOPICREQ  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_WILLTOPICREQ){
        if (_sendQ.getMessage(0)->getType() == MQTTS_TYPE_CONNECT){
            MqttsWillTopic mqMsg = MqttsWillTopic();
            mqMsg.setWillTopic(_willTopic);
            int index = _sendQ.addPriorityRequest((MqttsMessage*)&mqMsg);
            if ( index <= 0){
                _sendQ.setStatus(index, MQTTS_MSG_REQUEST);
            }else{
                *returnCode = MQTTS_ERR_OUT_OF_MEMORY;
            }
        }
        *returnCode = MQTTS_ERR_NO_ERROR;

/*---------  WILLMSGREQ  -----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_WILLMSGREQ  && _sendQ.getStatus(0) == MQTTS_MSG_WAIT_ACK){
        if (_sendQ.getMessage(0)->getType() == MQTTS_TYPE_WILLTOPIC){
            _sendQ.getMessage(0)->setStatus(MQTTS_MSG_COMPLETE);
            MqttsWillMsg mqMsg = MqttsWillMsg();
            mqMsg.setWillMsg(_willMessage);
            int index = _sendQ.addPriorityRequest((MqttsMessage*)&mqMsg);
            if ( index <= 0){
                _sendQ.setStatus(index, MQTTS_MSG_REQUEST);
            }else{
                *returnCode = MQTTS_ERR_OUT_OF_MEMORY;
            }
        }
        *returnCode = MQTTS_ERR_NO_ERROR;

    }else{
        *returnCode = MQTTS_ERR_NO_ERROR;
    }
}




void MqttsClient::delayTime(){
#ifdef ARDUINO
	srand(millis());
	delay(rand() % MQTTS_TIME_SEARCHGW);
#else
    srand((unsigned)time(NULL));
    usleep((rand() % MQTTS_TIME_SEARCHGW) * 1000);
#endif
}


/*-------------------------------------------------------------------------*/

int MqttsClient::exec(){
    if (_sendQ.getStatus(0) == MQTTS_MSG_COMPLETE){
        _sendQ.deleteRequest(0);
    }
    if ( _respTimer.isTimeUp() && _sendQ.getStatus(0) == MQTTS_MSG_WAIT_ACK){
    	_sendQ.deleteRequest(0);
    	_respTimer.stop();
    	return MQTTS_ERR_ACK_TIMEOUT;
    }

    if (_sendQ.getStatus(0) == MQTTS_MSG_REQUEST || _sendQ.getStatus(0) == MQTTS_MSG_RESEND_REQ){
        if (_gwHdl.isLost() || _gwHdl.isInit()){
            MqttsSearchGw mqttsMsg = MqttsSearchGw();
            mqttsMsg.setRadius(0);
            int index = _sendQ.addPriorityRequest((MqttsMessage*)&mqttsMsg);
            if ( index <= 0){
                _sendQ.setStatus(index, MQTTS_MSG_REQUEST);
            }else{
                return MQTTS_ERR_GATEWAY_LOST;
            }
            if (_sendQ.getMessage(0)->getType() == MQTTS_TYPE_SEARCHGW){
                _gwHdl.setStatus(MQTTS_GW_SEARCHING);
                return broadcast(_tSearch);
            }
        }else if (_gwHdl.isConnected()){
            return unicast(PACKET_TIMEOUT_MIN);
        }else if (_gwHdl.isDisconnected() || _gwHdl.isFound()){
            if (_sendQ.getMessage(0)->getType() == MQTTS_TYPE_CONNECT){
              return unicast(PACKET_TIMEOUT_MIN);
            }
        }
        return MQTTS_ERR_NOT_CONNECTED;
    }else{
    	if (_gwHdl.isPingRequired()){
    		return MQTTS_ERR_PING_REQUIRED;
    	}
        _zbee.readPacket(PACKET_TIMEOUT_MIN);  //  Just read packet, No send Request
        return MQTTS_ERR_NO_ERROR;
    }
}

int MqttsClient::broadcast(uint16_t packetReadTimeout){
    int retry = 0;
    while(retry < _nRetry){
       if (_zbee.bcastData(_sendQ.getMessage(0)->getFrameData(),
           _sendQ.getMessage(0)->getLength(), packetReadTimeout) == MQTTS_ERR_NO_ERROR){
           if (_qos){
               _respTimer.start(packetReadTimeout);
                   _sendQ.getMessage(0)->setStatus(MQTTS_MSG_WAIT_ACK);
           }else{
               _sendQ.getMessage(0)->setStatus(MQTTS_MSG_COMPLETE);
           }
           return MQTTS_ERR_NO_ERROR;
       }else{
           _sendQ.getMessage(0)->setStatus(MQTTS_MSG_SENDING);
           retry++;
       }
   }
   return MQTTS_ERR_RETRY_OVER;
}

int MqttsClient::unicast(uint16_t packetReadTimeout){
    int retry = _nRetryCnt = 0;
    int rc;

    while(retry < _nRetry){
        rc = _zbee.sendData(_gwHdl.getAddress64(), _gwHdl.getAddress16(), _sendQ.getMessage(0)->getFrameData(),
                            _sendQ.getMessage(0)->getLength(), packetReadTimeout);
        if (rc == PACKET_CORRECT){
            _gwHdl.setLastSendTime();
                if (_qos){
                    _respTimer.start(MQTTS_TIME_RESPONCE);
                    _sendQ.getMessage(0)->setStatus(MQTTS_MSG_WAIT_ACK);
                }else{
                    _sendQ.getMessage(0)->setStatus(MQTTS_MSG_COMPLETE);
                }
                return rc;
        }else if (rc == MQTTS_RC_REJECTED_CONGESTION){
            if (_sendQ.getMessage(0)->getType() == MQTTS_TYPE_PUBLISH ||
                _sendQ.getMessage(0)->getType() == MQTTS_TYPE_SUBSCRIBE){
                _sendQ.getMessage(0)->getBody()[0] = (_clientFlg || MQTTS_FLAG_DUP);
            }
            _sendQ.getMessage(0)->setStatus(MQTTS_MSG_RESEND_REQ);
            retry++;
        }else{
            return rc;
        }
    }
    return MQTTS_ERR_RETRY_OVER;
}


int MqttsClient::sendMsg(MqttsMessage* mqttsMsgPtr){
    int index = _sendQ.addRequest((MqttsMessage*)mqttsMsgPtr);
    if ( index >= 0){
        if (_sendQ.getStatus(index) != MQTTS_MSG_RESEND_REQ){
            _sendQ.setStatus(index, MQTTS_MSG_REQUEST);
        }
        return exec();
    }
    return MQTTS_ERR_CANNOT_ADD_REQUEST;
}

int MqttsClient::connect(){   //  0: okZB_CLIENT  -1:Not find  -2:Not disconnected
    MqttsConnect mqttsMsg = MqttsConnect();
    mqttsMsg.setDuration(_gwHdl.getKeepAlive());
    mqttsMsg.setFlags(_clientFlg);
    if (_clientId){
        mqttsMsg.setClientId(_clientId);
    }
    return sendMsg((MqttsMessage*)&mqttsMsg);
}

int MqttsClient::publish(const char* topic, char* data, int dataLength, Callback* errCallback){
  uint16_t topicId = _topics.getTopicId((char*)topic);
  topicId = 0x1234;   ////////////////////   DEBUG    ///////////////////////
  if (topicId){
      MqttsPublish mqttsMsg = MqttsPublish();
      mqttsMsg.setFlags(_clientFlg);
      mqttsMsg.setTopicId(topicId);
      mqttsMsg.setData((uint8_t*)data, (uint8_t)dataLength);
      if (!_qos){
          mqttsMsg.setMsgId(getNextMsgId());
      }
      return sendMsg((MqttsMessage*)&mqttsMsg);
  }
  return MQTTS_ERR_NO_TOPICID;
}

int MqttsClient::registerTopic(char* topic){
    MqttsRegister mqttsMsg = MqttsRegister();
    mqttsMsg.setTopicName(topic);
    if (!_qos){
	    mqttsMsg.setMsgId(getNextMsgId());
	}
    return sendMsg((MqttsMessage*)&mqttsMsg);
}

int MqttsClient::subscribe(char* topic, Callback* callback){
    MqttsSubscribe mqttsMsg = MqttsSubscribe();
    uint16_t topicId = _topics.getTopicId((char*)topic);
    if (topicId){
        mqttsMsg.setTopicId(topicId);
    }else{
        mqttsMsg.setTopicName(topic);
    }
    mqttsMsg.setFlags(_clientFlg);
    if (!_qos){
	    mqttsMsg.setMsgId(getNextMsgId());
	}
    return sendMsg((MqttsMessage*)&mqttsMsg);
}

int MqttsClient::unsubscribe(char* topic){
    MqttsUnsubscribe mqttsMsg = MqttsUnsubscribe();
    uint16_t topicId = _topics.getTopicId((char*)topic);
    if (topicId){
        mqttsMsg.setTopicId(topicId);
    }else{
        mqttsMsg.setTopicName(topic);
    }
    mqttsMsg.setFlags(_clientFlg);
    if (!_qos){
	    mqttsMsg.setMsgId(getNextMsgId());
	}
    return sendMsg((MqttsMessage*)&mqttsMsg);
}

int MqttsClient::disconnect(uint16_t duration){
    MqttsDisconnect mqttsMsg = MqttsDisconnect();
    if (duration){
        mqttsMsg.setDuration(duration);
    }
    return sendMsg((MqttsMessage*)&mqttsMsg);
}

int  MqttsClient::searchGw(uint8_t radius){
    MqttsSearchGw mqttsMsg = MqttsSearchGw();
    mqttsMsg.setRadius(radius);
    return sendMsg((MqttsMessage*)&mqttsMsg);
}

int  MqttsClient::pingReq(char* clientId){
    MqttsPingReq mqttsMsg = MqttsPingReq();
    if ( clientId != NULL){
        mqttsMsg.setClientId((const char*)clientId);
    }
    return sendMsg((MqttsMessage*)&mqttsMsg);
}

int MqttsClient::willTopic(){
    MqttsWillTopic mqttsMsg = MqttsWillTopic();
    mqttsMsg.setFlags(_clientFlg);
    mqttsMsg.setWillTopic(_willTopic);
    return sendMsg((MqttsMessage*)&mqttsMsg);
}

int MqttsClient::willMsg(){
    MqttsWillMsg mqttsMsg = MqttsWillMsg();
    mqttsMsg.setWillMsg(_willMessage);
    return sendMsg((MqttsMessage*)&mqttsMsg);
}

bool MqttsClient::init(){
        return _zbee.init(ZB_CLIENT);
}




/////////////////// End of File ///////////////
