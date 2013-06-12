

#ifdef ARDUINO
  #include <MQTTS_Defines.h>
  #include <MqttsClient.h>
#else
  #include "MQTTS_Defines.h"
  #include "MqttsClient.h"
#endif

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>


MQString* willtopic = new MQString("willtopic");
MQString* willmsg   = new MQString("willmsg");


int fnTp1(MqttsPublish* msg){
  printf("Execute fnTp1\n");
  return 0;
}




int main(int argc, char **argv){

  /* Client Test  */



  MqttsClient mqtts = MqttsClient();



    mqtts.begin(argv[1], B9600);
    mqtts.init("Node-02");
    mqtts.setQos(1);
    //mqtts.setWillTopic(willtopic);
    //mqtts.setWillMessage(willmsg);
    mqtts.setKeepAlive(60000);


    fprintf(stdout,"Connect\n");
    mqtts.connect();

    while(true){
        int rc = mqtts.execMsgRequest();
        if (mqtts.isGwConnected()){
            break;
        }else if ( rc != MQTTS_ERR_NO_ERROR){
            fprintf(stdout,"Rc = %d, Request status = 0x%x\n", rc, mqtts.getMsgRequestStatus());
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
            fprintf(stdout,"Rc = %d, Request status = 0x%x\n", rc, mqtts.getMsgRequestStatus());
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
        }else if( mqtts.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
            fprintf(stdout,"Rc = %d, Request status = 0x%x\n", rc, mqtts.getMsgRequestStatus());
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
            }else if( mqtts.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
                fprintf(stdout,"Rc = %d, Request status = 0x%x\n", rc, mqtts.getMsgRequestStatus());
                    mqtts.clearMsgRequest();
            }
        }
     */

    MQString *topic = new MQString("a/bcd/ef");

    mqtts.subscribe(topic, MQTTS_TOPIC_TYPE_NORMAL, fnTp1);

        while(true){
            int rc = mqtts.execMsgRequest();
            if (rc == MQTTS_ERR_NO_ERROR && mqtts.getMsgRequestCount() == 0){
                break;
            }else if( mqtts.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
                fprintf(stdout,"Rc = %d, Request status = 0x%x\n", rc, mqtts.getMsgRequestStatus());
                    mqtts.clearMsgRequest();
            }
        }

/*

        mqtts.unsubscribe(topic);

            while(true){
                int rc = mqtts.execMsgRequest();
                if (rc == MQTTS_ERR_NO_ERROR && mqtts.getMsgRequestCount() == 0){
                    break;
                }else if( mqtts.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
                    fprintf(stdout,"Rc = %d, Request status = 0x%x\n", rc, mqtts.getMsgRequestStatus());
                        mqtts.clearMsgRequest();
                }
            }
*/
        while(true){
                        int rc = mqtts.execMsgRequest();
                        if (rc == MQTTS_ERR_NO_ERROR && mqtts.getMsgRequestCount() == 0){
                            continue;
                        }else if( mqtts.getMsgRequestStatus() != MQTTS_MSG_REQUEST){
                            fprintf(stdout,"Rc = %d, Request status = 0x%x\n", rc, mqtts.getMsgRequestStatus());
                                mqtts.clearMsgRequest();
                        }
                    }

     fprintf(stdout,"__end__\n");


}
