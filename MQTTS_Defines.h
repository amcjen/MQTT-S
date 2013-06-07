#ifndef  MQTTS_DEFINES_H_
#define  MQTTS_DEFINES_H_

/*=================================
 * Device Type Default ARDUINO
 ==================================*/
#define LINUX
//#define MBED

//#define XBEE_FLOWCTL_CRTSCTS

/*=================================
 *    Debug Condition
 ==================================*/
#define DEBUG_ZBEESTACK
#define DEBUG_MQTTS


/*=================================
 *    Data Type
 ==================================*/
#ifndef ARDUINO
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long  uint32_t;
#endif



#endif   /* MQTTS_DEFINES_H_ */
