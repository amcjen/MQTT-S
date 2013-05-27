#ifndef  MQTTS_DEFINES_H_
#define  MQTTS_DEFINES_H_

/*=========================
 *    Debug Condition
 ==========================*/
#define DEBUG_ZBEESTACK
#define ZBEE_EMULATION


/*=========================
 *    Data Type
 ==========================*/
#ifndef ARDUINO
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;
#endif



#endif   /* MQTTS_DEFINES_H_ */
