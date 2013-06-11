MQTT-S
======

MQTT-S Client via XBee  (running on linux and Arduino)

  1. Supported functions
  
  QOS Level 0 and 1
  
  Automatic SEARCHGW, GWINFO, ADVERTISE
  
  Automatic WILLTOPICREQ, WILLTOPIC, WILLMSGREQ, WILLMSG
  
  Automatic PINGREQ, PINGRESP
  
  CONNECT, REGISTER, SUBSCRIBE, PUBLISH, UNSUBSCRIBE, DISCONNECT

  CONNACK, REGACK, SUBACK, PUBACK, UNSUBACK
  
  2. Module descriptions
  
  1) MqttsClientApp.cpp
  
  Client application sample which is used for debugging.

  2) MqttsClient.cpp
  
  MQTT-S Client Engine class. This is used by application.
  
  Usages are shown as follows.
  
  MqttsClient mqtts = MqttsClient();



    mqtts.begin(argv[1], B9600);  // argv[1] is a serial device for XBee. ex) /dev/ttyUSB0
    
    mqtts.init("Node-02");  // Get XBee's address64, short address and set XBee Node ID, 
    
    mqtts.setQos(1);  // set QOS level.  0 or 1
    
    mqtts.setWillTopic(willtopic);  // set WILLTOPIC. 
    
    mqtts.setWillMessage(willmsg);  // set WILLMSG  those are sent automatically.
    
    mqtts.setKeepAlive(60000);   // PINGREQ interval time
    
    mqtts.connect();   // CONNECT  when the gateway is lost, SEARCHGW is sent automatically.

    mqtts.subscribe(topic, callback); // when the subscribed topic's data is publish, execute the callback.
    
    mqtts.registerTopic(topic);  // Register topic and aquire a Topic ID
    
    mqtts.publish(topic, payload, payload_length); // publish the data, topic is converted into ID automatically.
    
    mqtts.unsubscribe(topic);
    
    mqtts.disconnect();
    
    3) MQTTS.cpp
    
    MQTT-S messages classes and some classes for client and Gateway.
    
    4) ZBeeStack.cpp
    
    XBee control classes base on https://code.google.com/p/xbee-arduino/ 
    
    5) Mqtts_Defines.h
    
    Default setting is for Arduino.  (comment out both systems)
    
    //#define LINUX 
    
    //#define MBED
    
    select the system and uncoment it.
   
   
  
  3. Configuration of XBee
  
  Serial interfacing
  
  [BD] 0-7           
  
  [D6] 0 or 1

  Other values are defaults. Baudrate is used by  mqtts.begin(device, baudrate) function.
  
  When you set D6 to 1, uncomment a line //#define XBEE_FLOWCTL_CRTSCTS in Mqtts_Defines.h
  
  
  
