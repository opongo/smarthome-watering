/*
    Copyright (c) 2016 Huy Tran <h.tran@opongo.com>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.
*/

#include <SPI85.h>
#include <Mirf.h>
#include <MirfHardwareSpi85Driver.h>

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <EEPROM.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// This USI was defined in SPI85.cpp
// Not to be confused with SPI (MOSI/MISO) used by ICSP pins
// Refer to page 61 of attiny84 datahseet
// USI pins could be redefined here
// #define USI-DO  5
// #define USI-DI  6
// #define USCK   4

#define CE    8
#define CSN   9

// YL-69 humidity sensor
#define YL_69_PIN A1
#define VCC_PIN A0

#define SENSOR_POWER_PIN 10

#define DEFAULT_PREFIX "01"

struct settings_t
{
  char rxAddr[6];
  int init;
} settings;

struct device_t
{
  long id;
  int init;
} device;

unsigned long time;
unsigned long m;
byte c;
int default_value;
char btString[100];
char command[100];
int btCounter;

void setup() {
  //  mySerial.begin(9600);
  //  mySerial.println("Hello, Smarthome-live.com");
  // clear();

  EEPROM.get(0, device);

  if (device.init != 1) {
    randomSeed(analogRead(A0));
    device.id = random(10000, 100000);
    device.init = 1;
    EEPROM.put(0, device);
  }

  EEPROM.get(sizeof(device), settings);

  char deviceBuff[5];
  ltoa (device.id, deviceBuff, 10);
  strcpy(settings.rxAddr, deviceBuff);

  settings.init = 0;

  // Init sensor
  pinMode(YL_69_PIN, INPUT);
  pinMode(VCC_PIN, INPUT);

  pinMode(SENSOR_POWER_PIN, OUTPUT);   // GND 0V

  Mirf.spi = &MirfHardwareSpi85;

  // Setup pins / SPI
  Mirf.cePin = CE;
  Mirf.csnPin = CSN;
  Mirf.init();

  //Configure reciving address.
  Mirf.setRADDR((byte *)settings.rxAddr);
  Mirf.setTADDR((byte *)"ma100");

  //Set the payload length to 1 byte

  Mirf.payload = 1;

  //auf 250kbit/s umstellen (0x26) (2Mbit => 0x0f)
  Mirf.configRegister(RF_SETUP, 0x26);

  //Write channel and payload config then power up reciver.
  Mirf.channel = 90;

  Mirf.config();

  digitalWrite(SENSOR_POWER_PIN, HIGH);
}


void loop() {
  if (Mirf.dataReady()) {
    Mirf.getData(&c);
    btString[btCounter++] = char(c);

    if (btString[btCounter - 1] == '\n') {
      btString[btCounter - 2] = '\0';

      if (strncmp(btString, "RF+", 3) == 0) {
        memcpy(command, btString + 3, sizeof(btString) - 3);

        if (strncmp(command, "DA", 2) == 0) {
          default_value = analogRead(VCC_PIN);
          int humidity = read_humidity_sensor();

          sprintf(btString, "S%04dE%04d", humidity, default_value);

          transmit(btString);
          resetRF();
        }

        if (strncmp(command, "DAS", 3) == 0) {
          deepsleep(-1);
        }

        if (strncmp(command, "OK", 2) == 0) {
          char id[5];
          memcpy( id, command + 2, sizeof(id));

          if (strncmp(id, settings.rxAddr, 5) == 0) {
            settings.init = 1;
            udpateSettings();
          }

          transmit(id);
        }

        if (strncmp(command, "CLEAR", 5) == 0) {
          clear();
        }
      }
      btString[0] = '\0';
      command[0] = '\0';
      btCounter = 0;
      time = millis();
    }
  } else {
    if (settings.init != 1) {
      setupRX();
    } else {
      checkTimeToSleep();
    }
  }
}

void resetRF() {
  Mirf.powerDown();
  Mirf.flushRx();
  Mirf.flushTx();
  delay(50);
  Mirf.powerUpRx();
  Mirf.powerUpTx();
  Mirf.config();
}

void clear() {
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}

void udpateSettings() {
  EEPROM.put(sizeof(device), settings);
}

void setupRX() {
  m = millis() - time;

  if (m >= 2000) {
    digitalWrite(SENSOR_POWER_PIN, LOW);
    delay(500);
    digitalWrite(SENSOR_POWER_PIN, HIGH);

    char commandBuff[13];
    memcpy(commandBuff, "RF+CRX01\0", 9);
    strcat(commandBuff, settings.rxAddr);

    transmit(commandBuff);
    time = millis();
  }
}

// sends a string via the nRF24L01
void transmit(char * string)
{
  for ( int i = 0; string[i] != 0x00 ; i++ )
  {
    c = byte(string[i]);
    Mirf.send(&c);
    // mySerial.println(c);
    while ( Mirf.isSending() ) ;
  }

  transmitlf();
}

// send a CR/LF sequence via the nRF24L01
void transmitlf(void)
{
  c = '\r';
  Mirf.send(&c);
  while ( Mirf.isSending() ) ;

  c = '\n';
  Mirf.send(&c);
  while ( Mirf.isSending() ) ;
}

float read_humidity_sensor() {
  delay(200);
  int value = analogRead(YL_69_PIN);
  return default_value - value;
}

// Watchdog timeout values
// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
// 6=1sec, 7=2sec, 8=4sec, 9=8sec
void setup_watchdog(int ii)
{
  // The prescale value is held in bits 5,2,1,0
  // This block moves ii itno these bits
  byte bb;
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);

  // Reset the watchdog reset flag
  MCUSR &= ~(1 << WDRF);
  // Start timed sequence
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // Set new watchdog timeout value
  WDTCSR = bb;
  // Enable interrupts instead of reset
  WDTCSR |= _BV(WDIE);
}

void checkTimeToSleep() {
  m = millis() - time;
  if (m >= 5000) {
    deepsleep(-1);
  }
}

void system_sleep()
{
  cbi(ADCSRA, ADEN);
  setup_watchdog(7);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
  sleep_enable();                          // enables the sleep bit in the mcucr register so sleep is possible
  sei();                               // Enable the Interrupts so the wdt can wake us up
  sleep_mode();                          // here the device is actually put to sleep!!

  sleep_disable();                       // first thing after waking from sleep: disable sleep...
  sbi(ADCSRA, ADEN);
}

// wait for totalTime ms
// the wait interval is to the nearest 4 seconds
void deepsleep(int waitTime)
{
  digitalWrite(SENSOR_POWER_PIN, LOW);

  // Calculate the delay time
  int waitCounter = 0;

  while (waitCounter != waitTime)
  {
    //    m = millis();
    Mirf.powerDown();

    system_sleep();

    Mirf.powerUpRx();
    Mirf.powerUpTx();
    Mirf.config();

    waitCounter++;
    char letter = '0';

    for (int i = 0; i < 30 || letter == '#'; i++) {
      if (Mirf.dataReady()) {
        Mirf.getData(&c);
        letter = char(c);
        if (letter == '#') {
          waitCounter = waitTime;
        }
      }
    }
  }

  time = millis();

  Mirf.flushRx();
  digitalWrite(SENSOR_POWER_PIN, HIGH);
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {}
