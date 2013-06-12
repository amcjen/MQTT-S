#include <MQTTS_Defines.h>
#include <MqttsClient.h>

#if  defined(DEBUG_MQTTS) || defined(DEBUG_ZBEESTACK) 
    #include <SoftwareSerial.h>
    uint8_t ssRX = 8; // Connect Arduino pin 8 to TX of usb-serial device
    uint8_t ssTX = 9; // Connect Arduino pin 9 to RX of usb-serial device
    SoftwareSerial debug(ssRX, ssTX);
#endif

#ifdef DEBUG_MQTTS
void debugPrint(int rc, uint8_t stat){
    debug.print("Rc = " );
    debug.print( rc, DEC);
    debug.print("  Request status = 0x");
    debug.println(stat, HEX);
    }
 #endif

    MqttsClient mqtts = MqttsClient();
    
    int  cb1(MqttsPublish* msg){
          #ifdef MQTTS_DEBUG
          debug.println("exec callback");
          #endif
          return 0;
    }
    
void setup(){
  #if  defined(DEBUG_MQTTS) || defined(DEBUG_ZBEESTACK) 
  debug.begin(19200);
#endif

  mqtts.begin(9600);
  mqtts.init("Node-02");
    mqtts.setQos(1);
    //mqtts.setWillTopic(willtopic);
    //mqtts.setWillMessage(willmsg);
    mqtts.setKeepAlive(60000);
    
}

void loop(){    

    #ifdef  DEBUG_MQTTS
    debug.println("Connect");
    #endif 
    mqtts.connect();

    while(true){
        int rc = mqtts.execMsgRequest();
        if (mqtts.isGwConnected()){
            break;
        }else if ( rc != MQTTS_ERR_NO_ERROR){
          #ifdef DEBUG_MQTTS
               debugPrint(rc, mqtts.getMsgRequestStatus());
           #endif
            if ( mqtts.getMsgRequestType() == MQTTS_TYPE_SEARCHGW){
                mqtts.setMsgRequestStatus(MQTTS_MSG_REQUEST);
            }else{
                mqtts.clearMsgRequest();
            }
        }
    }
/*
    MQString *topic = new MQString("a/bcd/ef");

    mqtts.registerTopic(topic);

    while(true){
            int rc = mqtts.execMsgRequest();
        if (rc == MQTTS_ERR_NO_ERROR && mqtts.getMsgRequestCount() == 0){
            break;
        }else if ( mqtts.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
            #ifdef DEBUG_MQTTS
               debugPrint(rc, mqtts.getMsgRequestStatus());
           #endif
                mqtts.clearMsgRequest();
        }
    }
*/
    /*
    mqtts.disconnect();
    while(true){
        int rc = mqtts.execMsgRequest();
        if (rc == MQTTS_ERR_NO_ERROR && mqtts.getMsgRequestCount() == 0){
            break;
        }else if  ( mqtts.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
            #ifdef DEBUG_MQTTS
               debugPrint(rc, mqtts.getMsgRequestStatus());
           #endif
                mqtts.clearMsgRequest();
        }
    }
    */

    /*
    mqtts.willTopic();

        while(true){
            int rc = mqtts.execMsgRequest();
            if (rc == MQTTS_ERR_NO_ERROR && mqtts.getMsgRequestCount() == 0){
                break;
            }else if  ( mqtts.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
               #ifdef DEBUG_MQTTS
               debugPrint(rc, mqtts.getMsgRequestStatus());
           #endif
                    mqtts.clearMsgRequest();
            }
        }
     */

    MQString *topic = new MQString("a/bcd/ef");

    mqtts.subscribe(topic, MQTTS_TOPIC_TYPE_NORMAL, cb1);

        while(true){
            int rc = mqtts.execMsgRequest();
            if (rc == MQTTS_ERR_NO_ERROR && mqtts.getMsgRequestCount() == 0){
                break;
            }else if  ( mqtts.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
                #ifdef DEBUG_MQTTS
               debugPrint(rc, mqtts.getMsgRequestStatus());
           #endif
                    mqtts.clearMsgRequest();
            }
        }

/*

        mqtts.unsubscribe(topic);

            while(true){
                int rc = mqtts.execMsgRequest();
                if (rc == MQTTS_ERR_NO_ERROR && mqtts.getMsgRequestCount() == 0){
                    break;
                }else if  ( mqtts.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
                    #ifdef DEBUG_MQTTS
                       debugPrint(rc, mqtts.getMsgRequestStatus());
                   #endif
                        mqtts.clearMsgRequest();
                }
            }
*/
        while(true){
                        int rc = mqtts.execMsgRequest();
                        if (rc == MQTTS_ERR_NO_ERROR && mqtts.getMsgRequestCount() == 0){
                            continue;
                        }else if  ( mqtts.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
                            #ifdef DEBUG_MQTTS
                                 debugPrint(rc, mqtts.getMsgRequestStatus());
                           #endif
                                mqtts.clearMsgRequest();
                        }
                    }
                    
while(true){}
  delay(1000);
}
