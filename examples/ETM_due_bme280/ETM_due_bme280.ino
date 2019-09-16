/* Sketch to demonstrate use of the Eseye Telemetry Module */

/* This sketch reads temperatures from a BME280 and publishes temperature, humidity and pressure via MQTT.
 * Publish topics are 'Temperature/<thingname>', 'Humidity/<thingname>' and 'Pressure/<thingname>'.
 * Subscription topic is 'update/<thingname>'. Publish a number to this topic to adjust the period (in seconds)
 * between reads of the BME280 and publishes of data.
 * 
 * prerequisites:
 * BG96 modem must be loaded with ETM software version 0.8.0 or higher
 * SIM must be Eseye Anynet Secure registered to an AWS account and provisioned.
 * 
 * Creation of AWSIoT thing can occur at any time as ETM will wait until the security credentials have propagated
 * to the SIM before trying to connect. 
 */

#include <Wire.h>
#include <BME280I2C.h>

/* Use this sketch with Arduino DUE, MikroElektronika Arduino MEGA click shield, LTE IoT2 click (slot 1) and Weather click (slot 2) */
/* Use the default serial port for debug/tracing */
/* Use Serial1 (Due/ATMega32u4) for the ETM */

#define WITH_DEBUGSERIAL

#define ATSERIAL Serial1
#define DEBUGSERIAL Serial

#include "eseyetelemetrymodule.h"

TwoWire *wire = &Wire;
#define NULL_SDA A4
#define NULL_SCL A5

BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

/* Modem defines */
#define MODEM_PWRKEY 49 //A3
#define MODEM_STAT   A0

eseyeETM myAWS(&ATSERIAL);

/* Ensure ETM is reset. Initialise uarts, ETM library and BME280. */
void setup() {
  pinMode(NULL_SDA, INPUT);
  pinMode(NULL_SCL, INPUT);
  wire->begin();
  
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

  if(!bme.begin()){
    DEBUGSERIAL.println("Error - no BME!");
    while(1);
  }
}

/* Default 5 minutes between reads */
#define TEMP_UPDATE_MILLIS 300000UL
int updatetime = TEMP_UPDATE_MILLIS;
int lasttempdisplay = 0;

/* Simple conversion of float to string */
void floattostring(char *str, float val, int fraclen){
  int val_int, val_fra;
  val_int = (int) val;
  val_fra = (int) ((val - (float)val_int) * fraclen);
  sprintf(str, "%d.%d", val_int, val_fra);
}

int pubtemp = -1;
int pubhum = -1;
int pubpres = -1;
int submsgidx = -1;

void readBMEData(){
    float temp(NAN), hum(NAN), pres(NAN);

    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_hPa);
    bme.read(pres, temp, hum, tempUnit, presUnit);  

    char opbuf[10];
    if(pubtemp != -1){
      floattostring(opbuf, temp, 10);
      DEBUGSERIAL.print("Pushtemp ");
      DEBUGSERIAL.print(opbuf);
      DEBUGSERIAL.println("C");
      myAWS.publishconfirm(pubtemp, 1, (uint8_t *)opbuf, strlen(opbuf));
    }
    if(pubhum != -1){
      floattostring(opbuf, hum, 10);
      DEBUGSERIAL.println("Pushhum");
      myAWS.publishconfirm(pubhum, 1, (uint8_t *)opbuf, strlen(opbuf));
    }
    if(pubpres != -1){
      floattostring(opbuf, pres, 100);
      DEBUGSERIAL.println("Pushpressure");
      myAWS.publishconfirm(pubpres, 1, (uint8_t *)opbuf, strlen(opbuf));
    }
}

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

  if(lasttempdisplay == 0 || (thismillis - lasttempdisplay > updatetime)){
    readBMEData();
    lasttempdisplay = thismillis;
  }

  if(isdone(myAWS.urcseen, ETM_IDLE)){
    if(etmstarted == false){
      etmstarted = true;
      /* Startup the anynet secure AWS pub/subs here */
      /* Start mqtt protocol */
      myAWS.startproto(ETM_MQTT);
      myAWS.waitSync();
    }
    if(mqttready == false && isdone(myAWS.urcseen, ETM_MQTT_RDY)){
      mqttready = true;
      pubtemp = myAWS.pubregconfirm((char *)"Temperature");
      pubhum = myAWS.pubregconfirm((char *)"Humidity");
      pubpres = myAWS.pubregconfirm((char *)"Pressure");
      if(pubtemp == -1 || pubhum == -1 || pubpres == -1){
          DEBUGSERIAL.println("Failed to register pubtopic");
      }
      /* Subscribe to update topic - the response is not timed as the suback will only be
       * received once ETM has established a connection with the broker and subscribed */
      submsgidx = myAWS.subscribeconfirm((char *)"update", subcb);
    }
  }
}
