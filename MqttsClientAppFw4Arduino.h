/*
 * MqttsClientApplication.h
 *
 *  Created on: 2013/05/27
 *      Author: Tomoaki Yamaguchi
 */

#ifndef MQTTSCLIENTAPPLICATION_H_
#define MQTTSCLIENTAPPLICATION_H_

#ifdef ARDUINO

#include <MqttsClient.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#define MQ_LED_PIN  13
#define MQ_INT0_PIN 2
#define MQ_SLEEP_PIN 3  // Connect to XBee DTR for hibernation mode
#define MQ_ERROR_RECOVERY_DURATION_ON 8
#define MQ_WAKEUP  0
#define MQ_SLEEP   1
#define MQ_ON      1
#define MQ_OFF     2

#define MQ_WDT_ERR   (B01100000)  // Error Indication time
//#define MQ_WDT_TIME  (B01000110)  // 1 Sec
//#define MQ_WDT_TIME_SEC   1       // 1 Sec

//#define MQ_WDT_TIME (B01000111)   // 2 Sec
//#define MQ_WDT_TIME_SEC   2       // 2 Sec

#define MQ_WDT_TIME (B01100000)     // 4 Sec
#define MQ_WDT_TIME_SEC   4         // 4 Sec  
 
//#define MQ_WDT_TIME (B01100001)   // 8 Sec
//#define MQ_WDT_TIME_SEC   8       // 8 Sec

#define MQ_WAKEUP_COUNT   5

typedef struct {
  uint16_t cnt;
	uint16_t cntReg;
	bool flg;
	void (*callback)(void);;
}MQ_TimerTbl;

enum MQ_INT_STATUS{ WAIT, INT0_LL, INT0_WAIT_HL, INT_WDT};

/*======================================
               Class WdTimer
========================================*/
class WdTimer {
public:
	WdTimer(void);
	WdTimer(uint8_t wdtTime);
	//void setup(uint8_t wdtTime);
	uint8_t registerCallback(uint16_t milisec, void (*proc)());
	void start(void);
	void stop(void);
	void timeUp(void);

private:	
	MQ_TimerTbl *_timerTbls;
	uint8_t _timerCnt;
	uint8_t _wdtTime;
	uint16_t _resolutionMilisec;
};

/*======================================
       Class MqttsClientApplication
========================================*/
class MqttsClientApplication{
public:
	MqttsClientApplication();
	~MqttsClientApplication();
	void registerInt0Callback(void (*callback)());
	void registerWdtCallback(uint16_t milisec, void (*callback)());
	void setup(const char* clientId, uint16_t baudrate);
	void checkInterupt();
	void sleepApp();
	void blinkIndicator(int msec);
    void setInterrupt();
	void wdtHandler();
	void interruptHandler();
	void sleepXB();
	void wakeupXB();
	uint8_t getMsgRequestType();
	uint8_t getMsgRequestStatus();
	uint8_t getMsgRequestCount();
	void clearMsgRequest();
	int  execMsgRequest();
	bool isGwConnected();
	void setMsgRequestStatus(uint8_t stat);

	int connect();
	int registerTopic(MQString* topic);
	int publish(MQString* topic, const char* data, int dataLength);
	int subscribe(MQString* topic, uint8_t type, TopicCallback callback);
	int unsubscribe(MQString* topic);
	int disconnect(uint16_t duration);

	void setKeepAlive(uint16_t msec);
	void setQos(uint8_t level);
	void setRetain(bool retain);
	void setClean(bool clean);
	void setClientId(MQString* id);
	void startWdt();
	void stopWdt();

private:
	MqttsClient _mqtts;
	bool _txFlag;
	uint8_t _wdtCnt;
	uint8_t _wakeupCnt;
	uint8_t _wakeupCntReg; 
	WdTimer _wdTimer;

	void (*_intHandler)(void);

};

extern MqttsClientApplication* theApplication;

#else

#endif /*ARDUINO*/




#endif /* MQTTSCLIENTAPPLICATION_H_ */
