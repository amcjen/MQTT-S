/*
 * MqttsClientFwApp.ino
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
 *  Created on: 2013/06/08
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 0.3.0
 *
 */
 
#include <MQTTS_Defines.h>
#include <MqttsClientAppFw4Arduino.h>


#if  defined(DEBUG_MQTTS) || defined(DEBUG_ZBEESTACK) 
// SoftwareSerial for consol
#include <SoftwareSerial.h>

uint8_t ssRX = 8; // Connect Arduino pin 8 to TX of usb-serial device
uint8_t ssTX = 9; // Connect Arduino pin 9 to RX of usb-serial device
SoftwareSerial debug(ssRX, ssTX);

void debugPrint(int rc, uint8_t stat){
    debug.print("Rc = " );
    debug.print( rc, DEC);
    debug.print("  Request status = 0x");
    debug.println(stat, HEX);
}
#endif

// Create Application
MqttsClientApplication app;

MQString* topic1 = new MQString("a/bcd/efg");
// Loopback


uint16_t  cnt0 = 0;
uint16_t  cnt1 = 0;

int pubCallback1(MqttsPublish* msg){
  debug.println("pubCallback1 was executed");
  app.blinkIndicator(1000);
  return 0;
}

// Callback for WDT
void wdtFunc0(){
  app.publish(topic1, "abcdefg", 6);
}

// Callbacks for WDT
void wdtFunc1(){
  app.publish(topic1, "123456", 6);
}

// Callback for INT0
void intFunc(){
  
  
}

/*
void func0() {

}
*/

void setup() {
#if  defined(DEBUG_MQTTS) || defined(DEBUG_ZBEESTACK) 
  debug.begin(19200);
#endif

  // Register Callback for INT0
  app.registerInt0Callback(intFunc);
  
  // Register Callbacks for WDT (millSec、callback)
  app.registerWdtCallback(10000,wdtFunc0);
  app.registerWdtCallback(20000,wdtFunc1);
  
  app.setup("Node-02",9600);
  app.setQos(1);
  app.setKeepAlive(60000);

}


void loop() {
   app.connect();

    while(true){
        int rc = app.execMsgRequest();
        if (app.isGwConnected()){
            break;
        }else if ( rc != MQTTS_ERR_NO_ERROR){
          #ifdef DEBUG_MQTTS
               debugPrint(rc, app.getMsgRequestStatus());
           #endif
            if ( app.getMsgRequestType() == MQTTS_TYPE_SEARCHGW){
                app.setMsgRequestStatus(MQTTS_MSG_REQUEST);
            }else{
                app.clearMsgRequest();
            }
        }
    }
    app.startWdt();
/*
    MQString *topic = new MQString("a/bcd/ef");

    app.registerTopic(topic);

    while(true){
            int rc = execMsgRequest();
        if (rc == MQTTS_ERR_NO_ERROR && app.getMsgRequestCount() == 0){
            break;
        }else if ( app.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
            #ifdef DEBUG_MQTTS
               debugPrint(rc, app.getMsgRequestStatus());
           #endif
                app.clearMsgRequest();
        }
    }
*/
    /*
    app.disconnect();
    while(true){
        int rc = app.execMsgRequest();
        if (rc == MQTTS_ERR_NO_ERROR && app.getMsgRequestCount() == 0){
            break;
        }else if  ( app.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
            #ifdef DEBUG_MQTTS
               debugPrint(rc, app.getMsgRequestStatus());
           #endif
                app.clearMsgRequest();
        }
    }
    */

    /*
    app.willTopic();

        while(true){
            int rc = app.execMsgRequest();
            if (rc == MQTTS_ERR_NO_ERROR && app.getMsgRequestCount() == 0){
                break;
            }else if  ( app.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
               #ifdef DEBUG_MQTTS
               debugPrint(rc, app.getMsgRequestStatus());
           #endif
                    app.clearMsgRequest();
            }
        }
     */

    MQString *topic = new MQString("a/bcd/ef");

    app.subscribe(topic, MQTTS_TOPIC_TYPE_NORMAL,  pubCallback1);

        while(true){
            int rc = app.execMsgRequest();
            if (rc == MQTTS_ERR_NO_ERROR && app.getMsgRequestCount() == 0){
                break;
            }else if  ( app.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
                #ifdef DEBUG_MQTTS
               debugPrint(rc, app.getMsgRequestStatus());
           #endif
                    app.clearMsgRequest();
            }
        }

/*

        app.unsubscribe(topic);

            while(true){
                int rc = app.execMsgRequest();
                if (rc == MQTTS_ERR_NO_ERROR && app.getMsgRequestCount() == 0){
                    break;
                }else if  ( app.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
                    #ifdef DEBUG_MQTTS
                       debugPrint(rc, app.getMsgRequestStatus());
                   #endif
                        app.clearMsgRequest();
                }
            }
*/
        while(true){
            int rc = app.execMsgRequest();
            if (rc == MQTTS_ERR_NO_ERROR && app.getMsgRequestCount() == 0){
                continue;
            }else if  ( app.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
                #ifdef DEBUG_MQTTS
                     debugPrint(rc, app.getMsgRequestStatus());
               #endif
                    app.clearMsgRequest();
            }
            app.checkInterupt();
            
        }

}




