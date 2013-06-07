

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



int main(int argc, char **argv){

  /* Client Test  */


    MqttsClient mqtts = MqttsClient();



    mqtts.begin(argv[1], B9600);
    mqtts.init("Node-02");
    mqtts.setQos(1);
    mqtts.setKeepAlive(10000);


    fprintf(stdout,"Connect\n");
    mqtts.connect();

    while(true){
        int rc = mqtts.execMsgRequest();
        if (rc){
            fprintf(stdout,"Rc = %d, Request type = 0x%x\n", rc, mqtts.getMsgRequestType());
            if ( mqtts.getMsgRequestType() == MQTTS_TYPE_SEARCHGW){
                sleep(5);
                mqtts.setMsgRequestStatus(MQTTS_MSG_REQUEST);
            }else{
                mqtts.clearMsgRequest();
            }
        }
}


     fprintf(stdout,"__end__\n");


}
