
/* Sketch to demonstrate use of the Eseye Telemetry Module */

/* This sketch demonstrates a simple mqtt publish/subscibe using ETM.
 * Publish topic is 'status/<thingname>'. A sequential 'Count <n>' is published to this topic at 5 minute intervals.
 * Subscription topic is 'update/<thingname>'. Publish a number to this topic to adjust the interval (in seconds).
 * 
 * prerequisites:
 * BG96 modem must be loaded with ETM software version 0.8.0 or higher
 * SIM must be Eseye Anynet Secure registered to an AWS account and provisioned.
 * 
 * Creation of AWSIoT thing can occur at any time as ETM will wait until the security credentials have propagated
 * to the SIM before trying to connect. 
 */

/* Use this sketch with Arduino DUE, MikroElektronika Arduino MEGA click shield and LTE IoT2 click (slot 1) */
/* Use the default serial port for debug/tracing */
/* Use Serial1 (Due/ATMega32u4) for the ETM */

#define WITH_DEBUGSERIAL

#define ATSERIAL Serial1
#define DEBUGSERIAL Serial

#include "eseyetelemetrymodule.h"

/* Modem defines */
#define MODEM_PWRKEY 49 
#define MODEM_STAT   A0

eseyeETM myAWS(&ATSERIAL);

/* Ensure ETM is reset. Initialise uarts and ETM library. */
void setup() {
  ATSERIAL.begin(115200);
  DEBUGSERIAL.begin(115200);
  DEBUGSERIAL.println("Starting ...");

  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_STAT, INPUT);
  
  if(digitalRead(MODEM_STAT) == HIGH){
    DEBUGSERIAL.println("Modem already powered - powering down");
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, LOW);
    // Spin waiting for modem to power down
    while(digitalRead(MODEM_STAT) != LOW);
  }
  
  /* Toggle the pwrkey on the modem */
  DEBUGSERIAL.println("Powering up modem");
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(500);
  digitalWrite(MODEM_PWRKEY, LOW);

#ifdef WITH_DEBUGSERIAL
  myAWS.init(NULL, &DEBUGSERIAL);
#else  
  myAWS.init();
#endif
}

/* Default 5 minutes between polls */
#define DEF_UPDATE_MILLIS 300000UL
int updatetime = DEF_UPDATE_MILLIS;
int lastpublish = 0;
int lastcount = 0;

int pubmsgidx = -1;
int submsgidx = -1;

void publish(){
    char msg[20];
    sprintf(msg, "Count %d", lastcount++);
    myAWS.publishconfirm(pubmsgidx, 1, (uint8_t *)msg, strlen(msg));
};

/* Accept commands from debug uart - currently allows AT commands to be sent using 'send ....' */
#ifdef WITH_DEBUGSERIAL
#define UART_RX_BUFSIZE 100
static uint8_t uartrxbuf[UART_RX_BUFSIZE + 1];
static unsigned char uartrxbufidx = 0;
void checkcmd(){
  char nextchar;
  while (DEBUGSERIAL.available() > 0) {
    nextchar = DEBUGSERIAL.read();

    DEBUGSERIAL.print(nextchar);

    uartrxbuf[uartrxbufidx++] = nextchar;
    uartrxbuf[uartrxbufidx] = 0;

    if(nextchar == '\n' || nextchar == '\r'){
      if(strncmp((char *)uartrxbuf, "send", 4) == 0){
        uartrxbuf[uartrxbufidx - 1] = 0;
        DEBUGSERIAL.print("Sending ");
        DEBUGSERIAL.print((char *)&uartrxbuf[5]);
        DEBUGSERIAL.println(" to modem");
        ATSERIAL.print((char *)&uartrxbuf[5]);
        ATSERIAL.print("\r\n");
      }
      uartrxbufidx = 0;
    }

    if(uartrxbufidx >= UART_RX_BUFSIZE)
      uartrxbufidx = 0;
  }
}
#endif

boolean etmstarted = false;
boolean mqttready = false;

/* Callback for "update" subscription */
void subcb(uint8_t *data, uint8_t length){
    updatetime = strtol((char *)data, NULL, 10);
    updatetime *= 1000;
    DEBUGSERIAL.print("Poll update ");
    DEBUGSERIAL.print(updatetime);
    DEBUGSERIAL.println(" mS");
}

void loop() {
  int thismillis = millis();

#ifdef WITH_DEBUGSERIAL
  checkcmd();
#endif
  
  myAWS.poll();

  if(lastpublish == 0 || (thismillis - lastpublish > updatetime)){
    publish();
    lastpublish = thismillis;
  }

  if(isdone(myAWS.urcseen, ETM_IDLE)){
    if(etmstarted == false){
      etmstarted = true;
      /* Start mqtt protocol */
      myAWS.startproto(ETM_MQTT);
      myAWS.waitSync();
    }
    if(mqttready == false && isdone(myAWS.urcseen, ETM_MQTT_RDY)){
      mqttready = true;
      pubmsgidx = myAWS.pubregconfirm((char *)"status");
      if(pubmsgidx == -1){
          DEBUGSERIAL.println("Failed to register pubtopic");
      }
      /* Subscribe to update topic - the response is not timed as the suback will only be
       * received once ETM has established a connection with the broker and subscribed */
      submsgidx = myAWS.subscribeconfirm((char *)"update", subcb);
    }
  }
}
