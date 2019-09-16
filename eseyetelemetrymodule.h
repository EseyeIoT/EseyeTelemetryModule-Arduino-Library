/***************************************************************************
  eseyetelemetrymodule library (v0.8)
  
  This library provides simplified access functions for the AT command 
  interface of Eseye Telemetry Module anynet-secure enabled modem-modules.
  
  The anynet-secure ETM AT interface is quite simple but this library
  tries to make it simpler still to use - specifically for subscribed
  topics by simply registering for topic subscriptions and supplying
  a callback function for received messages.
  
  Error-detection in pub/sub requests can also be enabled so the application
  can detect problems with its use of pub/sub indices.
  
  by Paul Tupper (01/2018) <paul.tupper@dataflex.com>
  
 ***************************************************************************/


#ifndef ESEYETELEMETRYMODULE_H__
#define ESEYETELEMETRYMODULE_H__

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

//#if !defined NEO_SW_SERIAL && !defined SOFTWARE_SERIAL
/* Software serial uses the software serial library */
//#define SOFTWARE_SERIAL
//#endif

/* FILTER_OK attempts to filter AT command responses from the ETM application 
 * while passing through responses to other AT commands from the application.
 * If the application is not using AT commands to the modem do not define 
 * FILTER_OK or set urccb as it will just waste program memory/cpu cycles. */
#define FILTER_OK

/* DEBUG_ESEYETELEMETRYMODULE adds debug trace support to the uart selected in the call to 
 * init(). If you are not using this it's best not to define DEBUG_ESEYETELEMETRYMODULE. */
#define DEBUG_ESEYETELEMETRYMODULE

/* TIMEOUT_RESPONSES waits for a period of time after sub/pub commands and 
 * marks the index as errored if no response has been seen. You cannot publish to 
 * an errored topic */
#define TIMEOUT_RESPONSES
#ifdef TIMEOUT_RESPONSES
#define PUB_TIMEOUT 3000UL /* 3 second timeout */
//#define SUB_TIMEOUT 3000UL /* 3 second timeout */
#endif

#define ESEYETELEMETRYMODULELIB_VERSION "0.8"

#define MAX_SUB_TOPICS 8
#define MAX_PUB_TOPICS 8		

/* Prototype for the AT command response callback function */
typedef void (*_atcb)(char *data);
/* Prototype for the message callback function */	
typedef void (*_msgcb)(uint8_t *data, uint8_t length);
/* Publish topic state */
typedef enum {PUB_TOPIC_ERROR = -1, PUB_TOPIC_NOT_IN_USE = 0, PUB_TOPIC_REGISTERING, PUB_TOPIC_REGISTERED, PUB_TOPIC_UNREGISTERING} tpubTopicState;
/* Subscribe topic state */
typedef enum {SUB_TOPIC_ERROR = -1, SUB_TOPIC_NOT_IN_USE = 0, SUB_TOPIC_SUBSCRIBING, SUB_TOPIC_SUBSCRIBED, SUB_TOPIC_UNSUBSCRIBING} tsubTopicState;
/* Reason for waking up/not sleeping (unable to sleep currently, timer, message from click board or external interrupt) */
typedef enum {TRY_AGAIN_SHORTLY, WAKE_TIMER, WAKE_CLICK, WAKE_INT} twakeReason;
/* Current state of connectivity */
typedef enum {ETM_UNKNOWN = -1, ETM_IDLE = 0, ETM_WAITKEYS, ETM_NETWORKSTART, ETM_SSLSTART, ETM_SSLCONN, ETM_MQTTSTART, ETM_MQTTREADY, ETM_MQTTSUB, ETM_UDPACTIVE, ETM_ERROR} tetmState;
/* Current state callback */
typedef void (*_statecb)(void);
/* Request state type */
typedef enum {ETM_STATE_ONCE = 0, ETM_STATE_ON, ETM_STATE_OFF} tetmRequestState;
typedef enum {ETM_MQTT, ETM_UDP} tetmProto;

/* Subscribed topic array element */	
struct subtpc{
  _msgcb messagecb;
  tsubTopicState substate;
};

/* Publish topic array element */
struct pubtpc{
  tpubTopicState pubstate;
#ifdef TIMEOUT_RESPONSES
  /* Include a senttime for each pub to enable timeout */
  unsigned long senttime;
#endif
};
				
class eseyeETM
{
public:
	eseyeETM(Stream *uart);
    
    void init(_atcb urccallback = NULL, Stream *trcuart = NULL);
    int startproto(tetmProto proto = ETM_MQTT);
    /* Subscribe topic API */
    int subscribe(char *topic, _msgcb callback);
    tsubTopicState substate(int idx);
    int unsubscribe(int idx);
#ifdef FILTER_OK
    int subscribeconfirm(char *topic, _msgcb callback);
#endif
    
    /* Publish topic API */
    int pubreg(char *topic);
    tpubTopicState pubstate(int idx);
    int pubunreg(int idx);
#ifdef FILTER_OK    
    int pubregconfirm(char *topic);
#endif
    
    /* Publish API */
    /* Non-atomic publish */
    int publish(int tpcidx, uint8_t qos, uint8_t *data, uint16_t datalen);
    boolean pubdone(void);
#ifdef FILTER_OK
    /* Atomic publish */
    int publishconfirm(int tpcidx, uint8_t qos, uint8_t *data, uint8_t datalen);
#endif
    
    /* Polling loop */
    void poll(void);

    /* Send AT command */
    void sendAT(char *atcmd);
    
#ifdef FILTER_OK
    bool inSync(void);
    void waitSync(void);
#endif
    
#define MODEM_SEEN    (0x01 << 0)
#define ETM_IDLE      (0x01 << 1)
#define ETM_MQTT_RDY  (0x01 << 2)
#define ETM_UDP_RDY   (0x01 << 3)
#define isdone(x,y) ((x & y) != 0)
    
    unsigned int urcseen;
    
    /* Request the current anynet-secure connectivity state */
    void updateState(tetmRequestState streq);
    /* Updated when required with the current anynet-secure state */
    tetmState currentstate;
    /* Register for state change callback */
    void statecb(_statecb statecb = NULL);
private:
    /* Callback function for unhandled URCs */
    _atcb atcallback;
    _statecb statecallback;
  
    struct subtpc subtopics[MAX_SUB_TOPICS];
    struct pubtpc pubtopics[MAX_PUB_TOPICS];

    Stream *atuart;
    Stream *dbguart;

    #define MODEM_RX_BUFSIZE 100
    uint8_t modemrxbuf[MODEM_RX_BUFSIZE];
    unsigned char rxbufidx;
    unsigned char binaryread;
    unsigned char buffered;
    uint8_t readingsub;
#ifdef FILTER_OK
    uint8_t outstanding_ok;
    void incOKreq(void);
#endif

    uint8_t clkslppin;
    uint8_t clkslppol;
    uint8_t hstwkpin;
    uint8_t hstwkpol;

#ifdef TIMEOUT_RESPONSES
    boolean checkTimeout(void);
#endif
#ifdef DEBUG_ESEYETELEMETRYMODULE
    void dbg_uart(const char* fmt, ...);
#endif

};
#endif // ESEYETELEMETRYMODULE_H

