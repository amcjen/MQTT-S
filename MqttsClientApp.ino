#include <MQTTS_Defines.h>
#include <MqttsClient.h>

#if  defined(DEBUG_MQTTS) || defined(DEBUG_ZBEESTACK) || defined(ZBEE_EMULATION)
    #include <SoftwareSerial.h>
    uint8_t ssRX = 8; // Connect Arduino pin 8 to TX of usb-serial device
    uint8_t ssTX = 9; // Connect Arduino pin 9 to RX of usb-serial device
    SoftwareSerial debug(ssRX, ssTX);
#endif

    MqttsClient mqtts = MqttsClient();
    
void setup(){
  #if  defined(DEBUG_MQTTS) || defined(DEBUG_ZBEESTACK) 
  debug.begin(19200);
#endif

  mqtts.begin(9600);
  mqtts.init("Node-02");
  mqtts.setQos(1);
    mqtts.setKeepAlive(10000);
    
    mqtts.connect();
    
}

void loop(){
     int rc = mqtts.execMsgRequest();
        if (rc){
            debug.print("Rc = ");
            debug.print(rc,DEC);
            debug.print(" Request Type = 0x");
            debug.println( mqtts.getMsgRequestType(),HEX);
            if ( mqtts.getMsgRequestType() == MQTTS_TYPE_SEARCHGW){
                delay(5000);
                mqtts.setMsgRequestStatus(MQTTS_MSG_REQUEST);
            }else{
                mqtts.clearMsgRequest();
            }
        }
  
}
