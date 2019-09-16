
/***************************************************************************
  Eseye Telemetry Module library (v0.8)
  
  This library provides simplified access functions for the AT command 
  interface of Eseye Telemetry Module anynet-secure enabled modem-modules.
  
  The anynet-secure click AT interface is quite simple but this library
  tries to make it even simpler to use - specifically for subscribed
  topics by simply registering for topic subscriptions and supplying
  a callback function for received messages.
  
  Error-detection in pub/sub requests can also be enabled so the application
  can detect problems with its use of pub/sub indices.
  
  by Paul Tupper (01/2018) <paul.tupper@dataflex.com>
  
 ***************************************************************************/

/* TODO:
 * store AT strings in PROGMEM to save ram.
 * profile variable usage to reduce RAM requirements
 * complete implementation of sleep support for battery-powered applications.
 */

#ifdef SOFTWARE_SERIAL
#include <SoftwareSerial.h>
#endif
#include "eseyetelemetrymodule.h"

#ifdef DEBUG_ESEYETELEMETRYMODULE
#define UARTDEBUG(x)   if(this->dbguart != NULL){ this->dbguart->print(x); }
#define UARTDEBUGLN(x) if(this->dbguart != NULL){ this->dbguart->println(x); }
#else
#define UARTDEBUG(x)
#define UARTDEBUGLN(x)
#endif

#ifdef DEBUG_ESEYETELEMETRYMODULE
#define MAX_DBG_LEN 50
void eseyeETM::dbg_uart(const char* fmt, ...){
    char tmp[MAX_DBG_LEN + 1];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, MAX_DBG_LEN, fmt, ap);
    va_end( ap );
    this->dbguart->print(tmp);
}
#define UARTDEBUGPRINTF(x...) if(this->dbguart != NULL){ this->dbg_uart(x); }
#else
#define UARTDEBUGPRINTF(x...) ;
#endif

//const char etm_start[]   = "AT+ETM";
const char etm_mqtt_start[] = "AT+EMQ";
const char etm_sub[]     = "SUBOPEN=";
const char etm_subcl[]   = "SUBCLOSE=";
const char etm_pub[]     = "PUBOPEN=";
const char etm_pubcl[]   = "PUBCLOSE=";
const char etm_publish[] = "PUBLISH=";
const char etm_sendok[]  = ":SEND OK";
const char etm_sendfail[] = ":SEND FAIL";

const char at_start[]     = "AT";
const char etm_urc[]      = "+ETM";
const char emq_urc[]      = "+EMQ";
const char etmrdyurc[]    = "IDLE";
const char emqrdyurc[]    = "EMQRDY";
const char eurdyurc[]     = "EURDY";
const char pub_start[]    = "PUB";
const char sub_start[]    = "SUB";
const char open_msg[]     = "OPEN";
const char close_msg[]    = "CLOSE";
const char state_msg[]    = "STATE";
#ifdef FILTER_OK
const char ok_msg[]       = "OK";
const char error_msg[]    = "ERROR";
const char crlf_msg[]     = "\r\n";
#endif
const char apprdy[]       = "APP RDY";

#ifdef FILTER_OK
void eseyeETM::incOKreq(){
    this->outstanding_ok++;
}

bool eseyeETM::inSync(void){
    return this->outstanding_ok != 0 ? false : true;
}

void eseyeETM::waitSync(void){
    while(this->inSync() != true){
        yield();
        this->poll();
    }
}
#endif

/* Subscribe topic API */

/* Subscribe to a topic using the first available topic index */
int eseyeETM::subscribe(char *topic, _msgcb callback){
  int topiccount = 0;
#ifdef TIMEOUT_RESPONSES
  this->checkTimeout();
#endif
  while(this->subtopics[topiccount].substate != SUB_TOPIC_NOT_IN_USE && this->subtopics[topiccount].substate != SUB_TOPIC_ERROR && topiccount < MAX_SUB_TOPICS){
    topiccount++;
  }
  if(topiccount == MAX_SUB_TOPICS)
    return -1;
  UARTDEBUGPRINTF("Subscribe to %s\n", topic);
  this->atuart->write(etm_mqtt_start);
  this->atuart->write(etm_sub);
  this->atuart->print(topiccount);
  this->atuart->write(",\"");
  this->atuart->write(topic);
  this->atuart->write("\"\r\n");
#ifdef FILTER_OK
  this->incOKreq();
#endif
  this->subtopics[topiccount].substate = SUB_TOPIC_SUBSCRIBING;
  this->subtopics[topiccount].messagecb = callback;
  return topiccount;
}

/* Have we successfully subscribed */
tsubTopicState eseyeETM::substate(int idx){
#ifdef TIMEOUT_RESPONSES
  this->checkTimeout();
#endif
  return this->subtopics[idx].substate;
}

#ifdef FILTER_OK
int eseyeETM::subscribeconfirm(char *topic, _msgcb callback){
    int res = this->subscribe(topic, callback);
    this->waitSync();
    return res;
}
#endif

/* Unsubscribe from a topic index */
int eseyeETM::unsubscribe(int idx){
#ifdef TIMEOUT_RESPONSES
  this->checkTimeout();
#endif
  if(idx >= MAX_SUB_TOPICS)
    return -1;
  if(this->subtopics[idx].substate == SUB_TOPIC_SUBSCRIBED){
    this->atuart->write(etm_mqtt_start);
    this->atuart->write(etm_subcl);
    this->atuart->print(idx);
    this->atuart->write("\r\n");
    this->subtopics[idx].substate = SUB_TOPIC_UNSUBSCRIBING;
#ifdef FILTER_OK
    this->incOKreq();
#endif  
    return 0;
  }
  return -1;
}

/* Publish topic API */

#define TOPIC_NOT_REGISTERED 0
#define TOPIC_REGISTERING    1
#define TOPIC_REGISTERED     2

/* Register a publish topic */
int eseyeETM::pubreg(char *topic){
  int topiccount = 0;
#ifdef TIMEOUT_RESPONSES
  this->checkTimeout();
#endif
  while(this->pubtopics[topiccount].pubstate != PUB_TOPIC_NOT_IN_USE && this->pubtopics[topiccount].pubstate != PUB_TOPIC_ERROR && topiccount < MAX_PUB_TOPICS){
    topiccount++;
  }
  if(topiccount == MAX_PUB_TOPICS)
    return -1;
  this->atuart->write(etm_mqtt_start);
  this->atuart->write(etm_pub);
  this->atuart->print(topiccount);
  this->atuart->write(",\"");
  this->atuart->write(topic);
  this->atuart->write("\"\r\n");
  UARTDEBUGPRINTF("Pubreg %s\n", topic);
#ifdef FILTER_OK
  this->incOKreq();
#endif
  this->pubtopics[topiccount].pubstate = PUB_TOPIC_REGISTERING;
#ifdef TIMEOUT_RESPONSES
  this->pubtopics[topiccount].senttime = millis();
#endif
  return topiccount;
}

/* Check if publish topic is registered */
tpubTopicState eseyeETM::pubstate(int idx){
#ifdef TIMEOUT_RESPONSES
  this->checkTimeout();
#endif
  return this->pubtopics[idx].pubstate;
}

/* Unregister a publish topic */
int eseyeETM::pubunreg(int idx){
#ifdef TIMEOUT_RESPONSES
  this->checkTimeout();
#endif
  if(idx >= MAX_PUB_TOPICS)
    return -1;
  if(this->pubtopics[idx].pubstate == PUB_TOPIC_REGISTERED){
    this->atuart->write(etm_mqtt_start);
    this->atuart->write(etm_pubcl);
    this->atuart->print(idx);
    this->atuart->write("\r\n"); 
    this->pubtopics[idx].pubstate = PUB_TOPIC_UNREGISTERING; 
#ifdef TIMEOUT_RESPONSES
    this->pubtopics[idx].senttime = millis();
#endif
#ifdef FILTER_OK
    this->incOKreq();
#endif
    return 0;
  }
  return -1;
}

/* Given an octext convert it to a two-character string ascii-hex representation */
static void octettohex(uint8_t octet, char *dest){
    uint8_t nibble = (octet >> 4) & 0x0f;
    if(nibble < 10)
        dest[0] = '0' + nibble;
    else
        dest[0] = 'A' + (nibble - 10);
    nibble = octet & 0x0f;
    if(nibble < 10)
        dest[1] = '0' + nibble;
    else
        dest[1] = 'A' + (nibble - 10);
    dest[2] = 0;
}

/* Publish a message to a topic by index */
int eseyeETM::publish(int tpcidx, uint8_t qos, uint8_t *data, uint16_t datalen){
    char hexbyte[3];
#ifdef TIMEOUT_RESPONSES
  this->checkTimeout();
#endif  
  /* TODO - check we're not already sending something */
  
  if(tpcidx < MAX_PUB_TOPICS && this->pubtopics[tpcidx].pubstate == PUB_TOPIC_REGISTERED){
    this->atuart->write(etm_mqtt_start);
    this->atuart->write(etm_publish);
    this->atuart->print(tpcidx);
    this->atuart->write(",");
    this->atuart->print(qos);
    this->atuart->write(",\"");    
    /* Convert data to ascii-hex */
    for(uint16_t countlen=0; countlen < datalen; countlen++){
        octettohex(data[countlen], hexbyte);
        this->atuart->write(hexbyte);
    }
    this->atuart->write("\"\r\n");
#ifdef FILTER_OK
    this->incOKreq();
#endif    
  }
  return 0;
}

/* Check if previous publish is complete */
boolean eseyeETM::pubdone(void){
  return true;
}

#ifdef FILTER_OK
int eseyeETM::pubregconfirm(char *topic){
    int pubtemp;
    tpubTopicState pstate = PUB_TOPIC_REGISTERING;
    pubtemp = this->pubreg(topic);
    while(pubtemp != -1 && pstate == PUB_TOPIC_REGISTERING){
        yield();
        this->poll();
        pstate = this->pubstate(pubtemp);
    }
    if(pstate == PUB_TOPIC_ERROR)
        pubtemp = -1;
    return pubtemp;
}
#endif

#ifdef FILTER_OK
int eseyeETM::publishconfirm(int tpcidx, uint8_t qos, uint8_t *data, uint8_t datalen){
    int res = this->publish(tpcidx, qos, data, datalen);
    this->waitSync();
    return res;
}
#endif

/* Polling loop - the work is done here */
void eseyeETM::poll(void){
  char nextchar;
#ifdef TIMEOUT_RESPONSES
  this->checkTimeout();
#endif 
  while (this->atuart->available() > 0) {
    nextchar = this->atuart->read();

#ifdef removed
    char tempstr[2];
    tempstr[0] = nextchar;
    tempstr[1] = 0;
    if(nextchar == 0x0d){
        UARTDEBUGLN(tempstr);
    }else{
        UARTDEBUG(tempstr);
    }
#endif

    this->modemrxbuf[this->rxbufidx++] = nextchar;
    this->modemrxbuf[this->rxbufidx] = 0;
    if(this->binaryread > 0){
      this->binaryread--;
      this->buffered++;      
      if(this->binaryread == 0){
        /* modemrxbuf contains the binary response/message */
        if(this->readingsub < MAX_SUB_TOPICS && this->subtopics[this->readingsub].messagecb != NULL){
          this->subtopics[this->readingsub].messagecb(this->modemrxbuf, this->buffered);
          this->buffered = 0;
          this->readingsub = 0xff; 
        }
        this->rxbufidx = 0;
      }
    }else{
      if(this->rxbufidx == 1 && nextchar == '>'){
        nextchar = 0;
      }
      
      /* Filter out leading CR/LF - an issue with BG96 */
      if(this->rxbufidx == 1 && (nextchar == 0x0d || nextchar == 0x0a)){
          this->rxbufidx = 0;
          continue;
      }
      
      if(nextchar == 0x0a){
          
        //UARTDEBUGPRINTF("Process %s\n", (char *)this->modemrxbuf);
          
        char *parseptr;
        boolean handled = false;
        /* This is the end of a response */
        if(strncmp((char *)this->modemrxbuf, etm_urc, strlen(etm_urc)) == 0){
            /* Handle module URCs */
            parseptr = (char *)&this->modemrxbuf[strlen(etm_urc)];          
            if(*parseptr == ':'){
                parseptr++;
                if(strncmp(parseptr, etmrdyurc, strlen(etmrdyurc)) == 0){
                    this->urcseen |= ETM_IDLE;
                    UARTDEBUGPRINTF("ETM running\n");
                    handled = true;
                }else if(strncmp(parseptr, emqrdyurc, strlen(emqrdyurc)) == 0){
                    this->urcseen |= ETM_MQTT_RDY;
                    UARTDEBUGPRINTF("MQTT ready\n");
                    handled = true;
                }else if(strncmp(parseptr, eurdyurc, strlen(eurdyurc)) == 0){
                    this->urcseen |= ETM_UDP_RDY;
                    UARTDEBUGPRINTF("UDP ready\n");
                    handled = true;
                }
            }else if(strncmp(parseptr, state_msg, strlen(state_msg)) == 0){
                this->currentstate = (tetmState)strtol(parseptr + 7, NULL, 10);
                if(this->statecallback != NULL)
                    this->statecallback();
                handled = true;
            }
        }else if(strncmp((char *)this->modemrxbuf, emq_urc, strlen(emq_urc)) == 0){
            /* Handle MQTT URCs */
            parseptr = (char *)&this->modemrxbuf[strlen(emq_urc)];
            if(strncmp(parseptr + 3, open_msg, strlen(open_msg)) == 0){
              char *errptr;
              /* Read the index */
              uint8_t idx = strtol(parseptr + 8, &errptr, 10);
              /* Read the error code */
              int8_t err = strtol(errptr + 1, NULL, 10);
              if(strncmp(parseptr, sub_start, strlen(sub_start)) == 0){
                UARTDEBUGPRINTF("subscribe %d err %d\n", idx, err);
                /* If we get an already subscribed error assume it was us from before a reboot */
                if(err == 0 || err == -2)
                  this->subtopics[idx].substate = SUB_TOPIC_SUBSCRIBED;
                else
                  this->subtopics[idx].substate = SUB_TOPIC_ERROR;
              }else if(strncmp(parseptr, pub_start, strlen(pub_start)) == 0){
                UARTDEBUGPRINTF("pubreg %d err %d\n", idx, err);
                /* If we get an already registered error assume it was us from before a reboot */
                if(err == 0 || err == -2)
                  this->pubtopics[idx].pubstate = PUB_TOPIC_REGISTERED;
                else
                  this->pubtopics[idx].pubstate = PUB_TOPIC_ERROR;    
              }
              handled = true;
            }else if(strncmp(parseptr + 3, close_msg, strlen(close_msg)) == 0){
              char *errptr;
              /* Read the index */
              uint8_t idx = strtol(parseptr + 9, &errptr, 10);
              /* Read the error code */
              int8_t err = strtol(errptr + 1, NULL, 10);
              if(strncmp(parseptr, sub_start, strlen(sub_start)) == 0){
                UARTDEBUGPRINTF("unsubscribe %d err %d\n", idx, err);
                //if(err == 0)
                  this->subtopics[idx].substate = SUB_TOPIC_NOT_IN_USE;
                //else
                //  subtopics[idx].substate = SUB_TOPIC_SUBSCRIBED;
              }else if(strncmp(parseptr, pub_start, strlen(pub_start)) == 0){
                UARTDEBUGPRINTF("pubunreg %d err %d\n", idx, err);
                //if(err == 0)
                  this->pubtopics[idx].pubstate = PUB_TOPIC_NOT_IN_USE;
                //else
                //  pubtopics[idx].pubstate = PUB_TOPIC_REGISTERED;
              }
              handled = true;
            }else if(*parseptr == ':'){
                parseptr++;
                /* This is a published message to which we are subscribed */
                char *lenptr;
                /* Read the subindex */
                uint8_t idx = strtol(parseptr, &lenptr, 10);
                /* Read the length */
                uint8_t len = strtol(lenptr + 1, NULL, 10);
                this->readingsub = idx;
                this->binaryread = len;
                handled = true;
            }
        }else if(strncmp((char *)this->modemrxbuf, etm_sendok, strlen(etm_sendok)) == 0){
          UARTDEBUGPRINTF("Send OK\n");
          handled = true;
        }else if(strncmp((char *)this->modemrxbuf, etm_sendfail, strlen(etm_sendfail)) == 0){
          UARTDEBUGPRINTF("Send Fail\n");
          handled = true;
        }
        /* Specially for BG96 - AT channel starts with echo true so we turn it off */
        else if(strncmp((char *)this->modemrxbuf, apprdy, strlen(apprdy)) == 0){
          this->atuart->write("ATE0\r\n");
          UARTDEBUGPRINTF("BG96 found\n");
          handled = true;
        }
#ifdef FILTER_OK
        else if(this->outstanding_ok > 0){
            if(strncmp((char *)this->modemrxbuf, ok_msg, strlen(ok_msg)) == 0){
              this->outstanding_ok--;
              handled = true;
            }else if(strncmp((char *)this->modemrxbuf, error_msg, strlen(error_msg)) == 0){
              this->outstanding_ok--;
              handled = true;
            }else if(strncmp((char *)this->modemrxbuf, crlf_msg, strlen(crlf_msg)) == 0){
              handled = true;
            }
        }
#endif
        if(handled == false){
          if(this->atcallback != NULL){
            //UARTDEBUGPRINTF("Forwarding %s\n", (char *)this->modemrxbuf);
            this->atcallback((char *)this->modemrxbuf);
          }else{
            UARTDEBUGPRINTF("Discarding %s\n", (char *)this->modemrxbuf);
          }
        }
        
        //this->modemrxbuf[this->rxbufidx] = 0;
        //UARTDEBUGPRINTF("Handled %s\n", (char *)this->modemrxbuf);
        
        this->rxbufidx = 0;
      }
    }
    if(this->rxbufidx >= MODEM_RX_BUFSIZE){
      this->rxbufidx = 0;
    }
  }
}

/* Send an AT command */
void eseyeETM::sendAT(char *atcmd){
#ifdef TIMEOUT_RESPONSES
    this->checkTimeout();
#endif
    this->atuart->print(atcmd);
}

void eseyeETM::updateState(tetmRequestState streq){
    if(streq == ETM_STATE_ONCE){
        this->currentstate = ETM_UNKNOWN;
        this->atuart->write("AT+ETMSTATE?\r\n");
    }else if(streq == ETM_STATE_ON){
        this->atuart->write("AT+ETMSTATE=1\r\n");
    }else{
        this->atuart->write("AT+ETMSTATE=0\r\n");
    }
#ifdef FILTER_OK
    this->incOKreq();
#endif
}

/* Create and initialise API */

eseyeETM::eseyeETM(Stream *uart){
    if(this->atuart == NULL)
        this->atuart = uart;
}

void eseyeETM::statecb(_statecb stateupdatecb){
    this->statecallback = stateupdatecb;
}

void eseyeETM::init(_atcb urccallback, Stream *trcuart) {
    int i;
    for(i = 0; i < MAX_SUB_TOPICS; i++){
        this->subtopics[i].messagecb = NULL;
        this->subtopics[i].substate = SUB_TOPIC_NOT_IN_USE;
    }
    for(i = 0; i < MAX_PUB_TOPICS; i++){
        this->pubtopics[i].pubstate = PUB_TOPIC_NOT_IN_USE;
    }

    this->atcallback = urccallback;
    this->dbguart = trcuart;
#ifdef BINARY_TRANSFER
    this->txbuflen = 0;
#endif
    this->rxbufidx = 0;
    this->binaryread = 0;
    this->buffered = 0;
    this->readingsub = 0xff;
    this->urcseen = 0;
    this->currentstate = ETM_UNKNOWN;
    this->statecallback = NULL;
}

int eseyeETM::startproto(tetmProto proto){
    if(proto == ETM_MQTT){
        this->atuart->write("AT+ETMSTATE=startmqtt\r\n");
    }else if (proto == ETM_UDP){
        this->atuart->write("AT+ETMSTATE=startudp\r\n");
    }else{
        return -1;
    }
#ifdef FILTER_OK
    this->incOKreq();
#endif
    return 0;
}

#ifdef TIMEOUT_RESPONSES
boolean eseyeETM::checkTimeout(void){
    /* Check pending pub/sub requests etc. */
    int topiccount = 0;
    boolean waiting = false;
    unsigned long now = millis(), diff;
    while(topiccount < MAX_PUB_TOPICS){
        diff = now - this->pubtopics[topiccount].senttime;
        if(this->pubtopics[topiccount].pubstate == PUB_TOPIC_REGISTERING || this->pubtopics[topiccount].pubstate == PUB_TOPIC_UNREGISTERING){
            if(diff >= PUB_TIMEOUT){
                this->pubtopics[topiccount].pubstate = PUB_TOPIC_ERROR;
                UARTDEBUGPRINTF("Pub idx %d timed out\n", topiccount);
            }else{
                waiting = true;
            }
        }
        topiccount++;
    }
    return waiting;
}
#endif




