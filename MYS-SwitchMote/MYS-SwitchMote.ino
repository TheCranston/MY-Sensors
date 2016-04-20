/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * Version 0.2 - 10/16/2015 - Ben Cranston & others noted below
 *
 * Inspired by and lifted code from:
 *  Felix Rusu  lowpowerlab.com  <-  Love his Moteinos
 *  from MySensors forum: http://forum.mysensors.org/user/awi
 *  www.projectsbykec.com  Kurt Clothier for the PWM dimmer code
 *  https://github.com/thomasfredericks/Bounce2  Thomas Fredericks for the debounce library
 * 
 * DESCRIPTION
 * This is the LowPowerLab SwitchMote porting to use MySensors for the communications library
 * Starting with the Switchmode code and then mixing in the MySensors Binary Switch sketch.
 * then extending the code to support dimmming of the status LEDs in mass. useful to reduce light contamination of sleeping areas.
 * Added a watchdog in case something goes really sideways
 */

// Enable debug prints to serial monitor
#define MY_DEBUG 
// Enable MY_DEBUG_VERBOSE flag for verbose debug prints. Requires DEBUG to be enabled.
// This will add even more to the size of the final sketch!
#define MY_DEBUG_VERBOSE

// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RF69_IRQ_PIN RF69_IRQ_PIN
#define MY_RF69_SPI_CS RF69_SPI_CS
#define MY_RF69_IRQ_NUM RF69_IRQ_NUM
#define MY_RFM69_NETWORKID     69
#define MY_RFM69_ENABLE_ENCRYPTION
//#define MY_RFM69_ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
//#define RFM69_ENCRYPTKEY MY_RFM69_ENCRYPTKEY

// Enables repeater functionality (relays messages from other nodes)
#define MY_REPEATER_FEATURE

// Flash leds on rx/tx/err
#define MY_LEDS_BLINKING_FEATURE
// Inverses the behavior of leds
#define MY_WITH_LEDS_BLINKING_INVERSE
// Set blinking period
#define MY_DEFAULT_LED_BLINK_PERIOD 50
#define MY_DEFAULT_ERR_LED_PIN 9  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  9  // Receive led pin
#define MY_DEFAULT_TX_LED_PIN  9  // the PCB, on board LED

#include <SPI.h>
#include <MySensor.h>
#include <Bounce2.h>
// #include "utility/SPIFlash.h"
// #include <avr/wdt.h>
// #include <EEPROM.h> 

#define SN "SwitchMote3"
#define SV "1.1.0"

// Define Children IDs
#define BUTTON1 1   // Id of the sensor child
#define BUTTON2 2
#define BUTTON3 3
#define RELAY 4
#define DIMMER 5

// Define Physical parts of the SwitchMote
#define BTNCOUNT            3  //1 or 3 (2 also possible)
#define BTNT                6  //digital pin of top button
#define BTNM                5  //digital pin of middle button
#define BTNB                4  //digital pin of bottom button
#define LED_RB             14  //digital pin for BOTTOM RED LED                     PC0
#define LED_RM             15  //digital pin for MIDDLE RED LED                     PC1
#define LED_RT             16  //digital pin for TOP RED LED                        PC2
#define LED_GB             17  //digital pin for BOTTOM GREEN LE                    PC3
#define LED_GM             18  //digital pin for MIDDLE GREEN LED                   PC4
#define LED_GT             19  //digital pin for TOP GREEN LED                      PC5
#define SSR0                7  //digital pin connected to Solid State Relay (SSR)
#define RELAY_ON 1
#define RELAY_OFF 0
#define ON                  1
#define OFF                 0

byte CurrentState[] = { false, false, false, false, 0 };
byte NewState[] = { false, false, false, false, 0 };
byte oldValue[] = { false, false, false, false };
byte children[] = { BUTTON1, BUTTON2, BUTTON3, RELAY, DIMMER };
byte pins[] = { BTNT, BTNM, BTNB, SSR0 };
byte RedLeds[] = { LED_RT, LED_RM, LED_RB };
byte GreenLeds[] = { LED_GT, LED_GM, LED_GB };
Bounce debouncer[] = { Bounce(), Bounce(), Bounce() };

int LED_DUTY_CYCLE = 1600 ;
int brightness = 1600 ; 
byte outputs = B00000111 ;  // Default all RED LEDs illuminated
byte btnIndex = 0;
byte stateIndex = 0;

// MySensor gw; 

MyMessage NodeMessages[] = { 
  MyMessage(BUTTON1,V_LIGHT),
  MyMessage(BUTTON2, V_LIGHT),
  MyMessage(BUTTON3, V_LIGHT),
  MyMessage(RELAY, V_LIGHT),
  MyMessage(DIMMER, V_PERCENTAGE)
};

// SPIFlash flash(8, 0xEF30);  //  So we can OTA update this SwitchMote

ISR(TIMER1_COMPA_vect){
  PORTC |= (outputs & 0x3F);         // Turn On LEDs, mask the outputs just to be sure
  OCR1B = brightness;               // Set the pulse width
}

ISR(TIMER1_COMPB_vect){
  PORTC &= 0xC0;                     // Turn Off LEDs - Leave the two top bits alone
}

void setup()  
{  
  // Whack the eeprom..  this is a workaround for borked nodes installed in the wall...
  // for (int i=0;i<512;i++) {
  //   EEPROM.write(i, 0xff);
  // }
  
  // Setup the LEDs, input pins and debouncer 
  for( byte i = 0; i < BTNCOUNT; i++)
  {
    pinMode(RedLeds[i], OUTPUT);
    pinMode(GreenLeds[i], OUTPUT);
    // pinMode(pins[i], INPUT_PULLUP);
    debouncer[i].attach(pins[i], INPUT_PULLUP);
    debouncer[i].interval(65);  //Anything less and we get false triggers on the other switches 
  }
  // Make sure relays are off when starting up (fixme: add logic to do multiple SSRs)
  pinMode(SSR0, OUTPUT);
  digitalWrite(SSR0, RELAY_OFF); // default off, just in case...

  // Set up Timer 1 for 125Hz LED pulse to control brightness
  PRR &= ~_BV(PRTIM1);                  // Enable Timer1 Clock
  TCCR1A = 0x00;                        // Outputs Disconnected
  TCCR1B = _BV(WGM12) |                 // CTC Mode, Top = OCR1A
           _BV(CS11) | _BV(CS10);       // Prescaler = 64
  OCR1A = 1999;                         // Top = (16MHz * 8ms / 64)-1
  OCR1B = LED_DUTY_CYCLE;               // LED PUlse Width
  TIMSK1 = _BV(OCIE1B) | _BV(OCIE1A);   // Interrupts Enable

  // wdt_enable(WDTO_1S);  //  Call over the dog..
}


void presentation() {
  // Send the Sketch Version Information to the Gateway
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SN, SV);

  // Register all sensors to gw (they will be created as child devices)
  present(BUTTON1, S_LIGHT);  // Top Button
  present(BUTTON2, S_LIGHT);  // Middle Button
  present(BUTTON3, S_LIGHT);  // Bottom Button
  present(RELAY, S_BINARY);    // SSR0 as sperate entity
  present(DIMMER, S_DIMMER);   // LED Dimmer funciton for indicators on SwitchMote
}


void loop() 
{
  // wdt_reset();  // pet the dog

  // process(); // tickle the MySensors framework  - depricated in 1.6 forward

  check_buttons();  // update logical state from button changes

  update_mote();  //  Apply any gathered state changes to the actual hardware
}

void check_buttons() {
 //on each loop pass check the next button
  btnIndex = (btnIndex + 1) % BTNCOUNT;
  if(debouncer[btnIndex].update()) {
    bool bncStat=debouncer[btnIndex].read();
    if(!bncStat && oldValue[btnIndex]) {
      NewState[btnIndex]=CurrentState[btnIndex]?false:true;
      oldValue[btnIndex]=false;
    } 
    if(bncStat && !oldValue[btnIndex]) {
      oldValue[btnIndex]=true;
    }
  } 
}

void update_mote() {
  //  This is where we make the incremental changes to the physical hardware
  stateIndex = (stateIndex + 1) % sizeof(children);
  if(NewState[stateIndex] != CurrentState[stateIndex]) {
    CurrentState[stateIndex] = NewState[stateIndex];
    if(stateIndex == 3) {
      digitalWrite(pins[stateIndex], CurrentState[stateIndex]);  // Case of the SSR0
    } else if(stateIndex == 4) {
      brightness = map(CurrentState[stateIndex],0,100,0,1600);        // Case for the Dimmer     
    } else {
      send(NodeMessages[stateIndex].set(CurrentState[stateIndex]?true:false), true);  // Send new state and request ack back
      if(CurrentState[stateIndex]) {
        bitClear(outputs, 2-stateIndex); // turn off the red
        bitSet(outputs, 5-stateIndex); // turn on the green
      } else {
        bitClear(outputs, 5-stateIndex); // Turn off the Green
        bitSet(outputs, 2-stateIndex);  // Turn on the Red
      }
    }
  }
}


void receive(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  if (!message.isAck()) {
    int ID = message.sensor; 
    if((ID >= BUTTON1) && (ID <= BTNCOUNT) && (message.type == V_STATUS)) {
      NewState[ID-1] = (byte)message.getBool()?true:false; // Change switch state
    }
    if(ID==RELAY) {
      NewState[ID-1] = (byte)message.getBool()?true:false; // Change relay state
    }
    if((ID == DIMMER)) {
      if(message.type == V_DIMMER) {
        NewState[ID-1] = (byte)constrain(message.getLong(),0,100); // if DIMMER type, adjust brightness
      } else if(message.type == V_STATUS) {
        NewState[ID-1] = (byte)message.getBool()?100:0;  // or if dimmer is toggled
      }
    }
  }
}