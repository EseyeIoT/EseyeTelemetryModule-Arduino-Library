/* Sketch to demonstrate use of the Eseye Telemetry Module */

/* This sketch reads temperatures from a BME280 and publishes temperature, humidity and pressure via MQTT.
 * Publish topics are 'Temperature/<thingname>', 'Humidity/<thingname>' and 'Pressure/<thingname>'.
 * Subscription topic is 'update/<thingname>'. Publish a number to this topic to adjust the period (in seconds)
 * between reads of the BME280 and publishes of data.
 * 
 * Status is displayed on a 96x39 pixel Mikroelektronika OLED B display
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
#include <avr/pgmspace.h>

/* Use this sketch with Arduino DUE, MikroElektronika Arduino MEGA click shield, LTE IoT2 click (slot 1), Weather click (slot 2) and OLED B click (i2c in slot 3) */
/* Use the default serial port for debug/tracing */
/* Use Serial1 (Due/ATMega32u4) for the ETM */

#define WITH_DEBUGSERIAL

#define ATSERIAL Serial1
#define DEBUGSERIAL Serial

#include "eseyetelemetrymodule.h"

/* Support for OLED B click board with 96x39 blue LED display */
#define SSD1306_96_39
#define SSD1306_LCDWIDTH                  96
#define SSD1306_LCDHEIGHT                 39

#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETMULTIPLEX 0xA8
#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETSTARTLINE 0x40
#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_SETSEGMENTREMAP 0xA1  // ja
#define SSD1306_SEGREMAP 0xA0

#define SSD1306_COMSCANDEC 0xC8
#define SSD1306_SETCOMPINS 0xDA
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_SETPRECHARGE 0xD9
#define SSD1306_SETVCOMDETECT 0xDB
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10
#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22
#define SSD1306_COMSCANINC 0xC0
#define SSD1306_SEGREMAP 0xA0
#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_DEACTIVATE_SCROLL 0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A

#define WIRE_MAX 32

int8_t i2caddr = 0x3C;
TwoWire *wire = &Wire;

#define NULL_SDA A4
#define NULL_SCL A5

unsigned char _x, _y, _sx=1, _sy=1; // scaling factors

#define OLED_RESET 47 //A2
#define OLED_DC 3

BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

/* Modem defines */
#define MODEM_PWRKEY 49 //A3
#define MODEM_STAT   A0

eseyeETM myAWS(&ATSERIAL);

/* font for OLED */
const unsigned char PROGMEM font[] = { // compact 5x8 font
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5F,0x00,0x00,0x00,0x07,0x00,0x07,0x00, //  'sp,!,"
    0x14,0x7F,0x14,0x7F,0x14, // #
    0x24,0x2A,0x7F,0x2A,0x12,0x23,0x13,0x08,0x64,0x62,0x36,0x49,0x56,0x20,0x50, //  '$,%,&
    0x00,0x08,0x07,0x03,0x00,0x00,0x1C,0x22,0x41,0x00,0x00,0x41,0x22,0x1C,0x00, //  '',(,)
    0x2A,0x1C,0x7F,0x1C,0x2A,0x08,0x08,0x3E,0x08,0x08,0x00,0x00,0x70,0x30,0x00, //  '*,+,,
    0x08,0x08,0x08,0x08,0x08,0x00,0x00,0x60,0x60,0x00,0x20,0x10,0x08,0x04,0x02, //  '-,.,/
    0x3E,0x51,0x49,0x45,0x3E,0x00,0x42,0x7F,0x40,0x00,0x72,0x49,0x49,0x49,0x46, //  '0,1,2
    0x21,0x41,0x49,0x4D,0x33,0x18,0x14,0x12,0x7F,0x10,0x27,0x45,0x45,0x45,0x39, //  '3,4,5
    0x3C,0x4A,0x49,0x49,0x31,0x41,0x21,0x11,0x09,0x07,0x36,0x49,0x49,0x49,0x36, //  '6,7,8
    0x46,0x49,0x49,0x29,0x1E,0x00,0x00,0x14,0x00,0x00,0x00,0x40,0x34,0x00,0x00, //  '9,:,;
    0x00,0x08,0x14,0x22,0x41,0x14,0x14,0x14,0x14,0x14,0x00,0x41,0x22,0x14,0x08, //  '<,=,>
    0x02,0x01,0x59,0x09,0x06,0x3E,0x41,0x5D,0x59,0x4E,                          //  '?,@
    0x7C,0x12,0x11,0x12,0x7C,                                                   //  'A
    0x7F,0x49,0x49,0x49,0x36,0x3E,0x41,0x41,0x41,0x22,0x7F,0x41,0x41,0x41,0x3E, //  'B,C,D
    0x7F,0x49,0x49,0x49,0x41,0x7F,0x09,0x09,0x09,0x01,0x3E,0x41,0x41,0x51,0x73, //  'E,F,G
    0x7F,0x08,0x08,0x08,0x7F,0x00,0x41,0x7F,0x41,0x00,0x20,0x40,0x41,0x3F,0x01, //  'H,I,J
    0x7F,0x08,0x14,0x22,0x41,0x7F,0x40,0x40,0x40,0x40,0x7F,0x02,0x1C,0x02,0x7F, //  'K,L,M
    0x7F,0x04,0x08,0x10,0x7F,0x3E,0x41,0x41,0x41,0x3E,0x7F,0x09,0x09,0x09,0x06, //  'N,O,P
    0x3E,0x41,0x51,0x21,0x5E,0x7F,0x09,0x19,0x29,0x46,0x26,0x49,0x49,0x49,0x32, //  'Q,R,S
    0x03,0x01,0x7F,0x01,0x03,0x3F,0x40,0x40,0x40,0x3F,0x1F,0x20,0x40,0x20,0x1F, //  'T,U,V
    0x3F,0x40,0x38,0x40,0x3F,0x63,0x14,0x08,0x14,0x63,0x03,0x04,0x78,0x04,0x03, //  'W,X,Y
    0x61,0x59,0x49,0x4D,0x43,                                                   //  'Z
    0x00,0x7F,0x41,0x41,0x41,0x02,0x04,0x08,0x10,0x20,                          //  '[,\'
    0x00,0x41,0x41,0x41,0x7F,0x04,0x02,0x01,0x02,0x04,0x40,0x40,0x40,0x40,0x40, //  '],^,_
    0x00,0x03,0x07,0x08,0x00,0x20,0x54,0x54,0x38,0x40,0x7F,0x28,0x44,0x44,0x38, //  '`,a,b
    0x38,0x44,0x44,0x44,0x28,0x38,0x44,0x44,0x28,0x7F,0x38,0x54,0x54,0x54,0x18, //  'c,d,e
    0x00,0x08,0x7E,0x09,0x02,0x0C,0x52,0x52,0x4A,0x3C,0x7F,0x08,0x04,0x04,0x78, //  'f,g,h
    0x00,0x44,0x7D,0x40,0x00,0x20,0x40,0x40,0x3D,0x00,0x7F,0x10,0x28,0x44,0x00, //  'i,j,k
    0x00,0x41,0x7F,0x40,0x00,0x7C,0x04,0x78,0x04,0x78,0x7C,0x08,0x04,0x04,0x78, //  'l,m,n
    0x38,0x44,0x44,0x44,0x38,0x7C,0x18,0x24,0x24,0x18,0x18,0x24,0x24,0x18,0x7C, //  'o,p,q
    0x7C,0x08,0x04,0x04,0x08,0x48,0x54,0x54,0x54,0x24,0x04,0x04,0x3F,0x44,0x24, //  'r,s,t
    0x3C,0x40,0x40,0x20,0x7C,0x1C,0x20,0x40,0x20,0x1C,0x3C,0x40,0x30,0x40,0x3C, //  'u,v,w
    0x44,0x28,0x10,0x28,0x44,0x4C,0x50,0x50,0x50,0x3C,0x44,0x64,0x54,0x4C,0x44, //  'x,y,z
    0x00,0x08,0x36,0x41,0x00,0x00,0x00,0x77,0x00,0x00,0x00,0x41,0x36,0x08,0x00, //  '{,|,}
    0x02,0x01,0x02,0x04,0x02                                                    //  '~
};

#define START_OLED_I2C wire->setClock(400000UL);
#define END_OLED_I2C wire->setClock(100000UL);

void OLED_M_command1(uint8_t c) {
    wire->beginTransmission(i2caddr);
    wire->write((uint8_t)0x00); // Co = 0, D/C = 0
    wire->write(c);
    wire->endTransmission();
}

void OLED_M_command(uint8_t c) {
  START_OLED_I2C
  OLED_M_command1(c);
  END_OLED_I2C
}

void OLED_M_writedata(uint8_t *bytes, int len){
  START_OLED_I2C
  wire->beginTransmission(i2caddr);
  wire->write((uint8_t)0x40);
  uint8_t bytesOut = 1;
  while(len--) {
    if(bytesOut >= WIRE_MAX) {
      wire->endTransmission();
      wire->beginTransmission(i2caddr);
      wire->write((uint8_t)0x40);
      bytesOut = 1;
    }
    wire->write(*bytes++);
    bytesOut++;
  }
  wire->endTransmission();
  END_OLED_I2C
}

void OLED_M_data(uint8_t byte){
  START_OLED_I2C
  wire->beginTransmission(i2caddr);
  wire->write((uint8_t)0x40);
  wire->write(byte);
  wire->endTransmission();
  END_OLED_I2C
}

// intialise OLED
void OLED_M_Init() {
#ifdef OLED_DC
  /* DC must be LOW */
  pinMode(OLED_DC, OUTPUT);
  digitalWrite(OLED_DC,LOW);
#endif
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, HIGH);
  delay(1);
  digitalWrite(OLED_RESET, LOW);
  delay(10);
  digitalWrite(OLED_RESET, HIGH);
  delay(10);
  START_OLED_I2C
  OLED_M_command1(SSD1306_DISPLAYOFF);             //0xAE  Set OLED Display Off
  OLED_M_command1(SSD1306_SETDISPLAYCLOCKDIV);     //0xD5  Set Display Clock Divide Ratio/Oscillator Frequency
  OLED_M_command1(0x80);
  OLED_M_command1(SSD1306_SETMULTIPLEX);           //0xA8  Set Multiplex Ratio
  OLED_M_command1(0x27);
  OLED_M_command1(SSD1306_SETSEGMENTREMAP);
  OLED_M_command1(SSD1306_COMSCANDEC);
  OLED_M_command1(SSD1306_SETDISPLAYOFFSET);       //0xD3  Set Display Offset
  OLED_M_command1(0x3f);                           // COM offset of -1 for this display ?????
  OLED_M_command1(SSD1306_CHARGEPUMP);             //0x8D  Set Charge Pump
  OLED_M_command1(0x14);
  OLED_M_command1(SSD1306_SETSTARTLINE);           //0x40  Set Display Start Line
  OLED_M_command1(SSD1306_SETCOMPINS);             //0xDA  Set COM Pins Hardware Configuration
  OLED_M_command1(0x12);
  OLED_M_command1(SSD1306_SETCONTRAST);            //0x81   Set Contrast Control
  OLED_M_command1(0xAF);
  OLED_M_command1(SSD1306_SETPRECHARGE);           //0xD9   Set Pre-Charge Period
  OLED_M_command1(0x25);
  OLED_M_command1(SSD1306_SETVCOMDETECT);          //0xDB   Set VCOMH Deselect Level
  OLED_M_command1(0x20);
  OLED_M_command1(SSD1306_DISPLAYALLON_RESUME);    //0xA4   Set Entire Display On/Off
  OLED_M_command1(SSD1306_NORMALDISPLAY);          //0xA6   Set Normal/Inverse Display
  OLED_M_command1(SSD1306_DISPLAYON);
  END_OLED_I2C
}

void OLED_SetRow(unsigned char add) {
    add = 0xB0 | add;
    OLED_M_command(add);
}

void OLED_SetColumn(unsigned char add) {
    add += 32;
    OLED_M_command((SSD1306_SETHIGHCOLUMN | (add >> 4))); // SET_HIGH_COLUMN
    OLED_M_command((0x0f & add));        // SET LOW_COLUMN
}

void OLED_SetScale(unsigned char sx, unsigned char sy) {
    _sx = sx; _sy = sy;
}

void OLED_Clear(void) {
    unsigned char i,j;
    for(i=0; i<5; i++) { // 5*8=40 pixel rows (actually 39)
        OLED_SetRow(i);
        OLED_SetColumn(0);
        for(j=0; j<96; j++)  OLED_M_data(0);
    }
    _x = 0; _y = 0;
    OLED_SetRow(0);
    OLED_SetColumn(0);
}

void OLED_Clearline(char y){
    OLED_SetRow(y);
    OLED_SetColumn(0);
    for(int j=0; j<96; j++)  OLED_M_data(0);
    OLED_SetRow(_y);
    OLED_SetColumn(_x);
}

/*  scalable horizontally and vertically */
void OLED_Putchar(char ch) {
    unsigned char i, j, k, byte;
    //const unsigned char *f = &font[(ch-' ')*5];
    const unsigned char mask[]={1, 3, 7, 0xf };

    int fontoffset = (ch-' ')*5;

    for(i=0; i<6; i++) {
        unsigned long word = 0;
        //byte = *f++ << 1;
        byte = pgm_read_byte_near(font + fontoffset++);
        if (i==5) byte = 0;
        for(j=0; j<8; j++) { // expand byte to word
            word <<= _sy;
            if (byte & 0x80) word |= mask[_sy-1];
            byte <<= 1;
        }
        for(j=0; j<_sy; j++) { // exp vert
            OLED_SetRow(_y+j) ;
            OLED_SetColumn(_x+i*_sx);
            for(k=0; k<_sx; k++) { // exp horiz
                OLED_M_data(word);
            }
            word >>= 8;
        }
    }

    _x+= 6 * _sx;
    if (_x >= SSD1306_LCDWIDTH) { // wrap x
        _x = 0; OLED_SetColumn(0);
        _y += _sy;
        if (_y >= 5-_sy) { // wrap y
            _y = 0;
        }
        OLED_SetRow(_y);
    }
}

void OLED_Putnext(char *s) {
    while(*s) {
        OLED_Putchar(*s++);
    }
}

/* Write a string to the OLED */
void OLED_Puts(char x, char y, char *s) {
    _y = y; _x = x;
    OLED_SetRow(_y);
    OLED_SetColumn(_x);
    OLED_Putnext(s);
}


/* Ensure ETM is reset. Initialise OLED, uarts, ETM library and BME280. */
void setup() {
  pinMode(NULL_SDA, INPUT);
  pinMode(NULL_SCL, INPUT);
  
  wire->begin();
  OLED_M_Init();
  OLED_SetRow(0);
  OLED_SetColumn(0);
  for(int rows = 0; rows < 5; rows ++)
    OLED_Clearline(rows);
  
  OLED_Puts(0,0, (char *)"AWS Weather");

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
    OLED_Puts(0,2,(char *)"NO BME!!!    ");
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

/* Print float to OLED */
void dispfloat(float val, int fraclen){
  char strbuf[8];
  floattostring(strbuf, val, fraclen);
  OLED_Putnext(strbuf);
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
    OLED_Puts(0, 2, (char *)"Temp ");
    dispfloat(temp, 10);
    OLED_Putnext((char *)" C");
    OLED_Puts(0, 3, (char *)"Hum  ");
    dispfloat(hum, 10);
    OLED_Putnext((char *)"% RH");
    OLED_Puts(0, 4, (char *)"Pres ");
    dispfloat(pres, 100);
    OLED_Putnext((char *)" hPa");  

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

const char *ETMStates[] = {"IDLE   ", "GETKEYS", "NETSTRT", "SSLSTRT", "SSLCONN", "MQTTGO ", "MQTTRDY", "MQTTSUB", "UDPAct ", "ERROR  "};

/* Callback for "update" subscription */
void subcb(uint8_t *data, uint8_t length){
    char dispnum[5]; 
    updatetime = strtol((char *)data, NULL, 10);
    OLED_Puts(0,1, (char *)"poll ");
    sprintf(dispnum, "%d", updatetime);
    OLED_Putnext(dispnum);
    OLED_Putnext((char *)" S");
    updatetime *= 1000;
    DEBUGSERIAL.print("Poll update ");
    DEBUGSERIAL.print(updatetime);
    DEBUGSERIAL.println(" mS");
}

/* Callback for state change update from ETM */
void state_changed(void){
    if(myAWS.currentstate != ETM_UNKNOWN){
        OLED_Puts(0,0,(char *)"State: ");
        OLED_Putnext((char *)ETMStates[myAWS.currentstate]);
    }
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
      OLED_Puts(0,0, (char *)"Modem ready");

      /* Start mqtt protocol */
      myAWS.startproto(ETM_MQTT);
      myAWS.waitSync();
      
      /* Register for state change callback */
      myAWS.statecb(state_changed);
      myAWS.updateState(ETM_STATE_ON);
      myAWS.waitSync();
    }
    if(mqttready == false && isdone(myAWS.urcseen, ETM_MQTT_RDY)){
      mqttready = true;
      pubtemp = myAWS.pubregconfirm((char *)"Temperature");
      pubhum = myAWS.pubregconfirm((char *)"Humidity");
      pubpres = myAWS.pubregconfirm((char *)"Pressure");
      if(pubtemp == -1 || pubhum == -1 || pubpres == -1){
          OLED_Puts(0,0,(char *)"ETM error!  ");
          DEBUGSERIAL.println("Failed to register pubtopic");
      }
      /* Subscribe to update topic - the response is not timed as the suback will only be
       * received once ETM has established a connection with the broker and subscribed */
      submsgidx = myAWS.subscribeconfirm((char *)"update", subcb);
    }
  }
}
