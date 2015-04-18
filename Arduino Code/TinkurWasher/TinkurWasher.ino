//////////////////////////////////////
//  Adam Zolyak
//  TinkurLab
//  www.TinkurLab.com
//  2014-2015
//
//  This software is released under the GNU GENERAL PUBLIC LICENSE Version 3
//
//  This project makes use the of the following libraries and contributions by others:
//
//  Adafruit CC3000 Library and Sample Code https://github.com/adafruit/Adafruit_CC3000_Library
//  =====
//  Copyright (c) 2013-2014 
//  Limor Fried, Kevin Townsend for Adafruit Industries & Tony DiCola (tony@tonydicola.com)
//  
//  Adafruit invests time and resources providing this open source code, 
//  please support Adafruit and open-source hardware by purchasing 
//  products from Adafruit!
//  
//  All rights reserved.
//  
//  =====
//  
//  EEPROM Write Anything Library and Sample Code http://playground.arduino.cc/Code/EEPROMWriteAnything
//
//  CC3000 Watchdog Timer Sample Code https://github.com/openhomeautomation/arduino-cc3000-xively/blob/master/CC3000_xively/CC3000_xively.ino
//
//  Running Median Library and Sample Code https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningMedian
//
//////////////////////////////////////

//  Notes
//  1) Must disconnect WiFi shield to program due to reset pin

#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"
#include "RunningMedian.h" //for median
#include <avr/wdt.h> // for watchdog timer
#include <EEPROM.h> // for saving settings between restarts
#include "EEPROMAnything.h" // for saving settings between restarts

#define APIKEY         "YOUR XIVELY API KEY" // Xivelt AUPI Key
#define FEEDID         "YOUR XILVELY FEED ID" // Xively Feed ID
#define USERAGENT      "TinkurWash" // User Agent

//Setup Adafruit CC3000 WiDFi Shield
// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    8 //had to cut CC3000 shield connection between pin 10 and WCS and jumper WCS to pin 8, see http://forums.adafruit.com/viewtopic.php?f=25&p=214928
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIV2); // you can change this clock speed

#define WLAN_SSID       "YOUR WIFI SSID" //WiFi Network SSID
#define WLAN_PASS       "YOUR WIFI PASSWORD" //WiFi Network Password 

// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2 //WiFi Network Security Type

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.

// What page to grab!
#define WEBSITE      "api.xively.com"


/**************************************************************************/
/*!
    @brief  Sets up the HW and the CC3000 module (called automatically
            on startup)
*/
/**************************************************************************/

uint32_t ip;

volatile int counter;   // for watchdog timer
volatile int countmax = 8;

int resetPin = 7;

//TinkurWash

struct config_t
{
    int selfrestart;
    int tiltSensorStatus;
    int dishwasherState;
    long knockSensor1Baseline;
    long tiltSensor1Baseline;
    long knockSensorToggleTime;
    long tiltSensorToggleTime;
    long endWashToggleTime;
} configuration;

int networkConnectionFailTimer = 0;
int dhcpTimeoutTimer = 0;

boolean loggingKnocks = false;
boolean verboseLoggingEnabled = true;

const int intervalSeconds = 1;
const int sampleSize = 10;

int doorOpenCleanSeconds = 60; //suggest 60
int washCompleteSeconds = 300; //suggest 300
int dryCompleteSeconds = 3600; //suggest 3600

const int xInput = A0;
const int yInput = A1;
const int zInput = A2;

long knockSensor1Baseline = 0;
long knockSensor1Interval = 0;
long knockSensor1Median = 0;
RunningMedian knockSensor1Last10 = RunningMedian(10);
long knockSensorLastToggleTime = 0;
long knockSensorToggleTime = 0;
long knockSensorToggleTimeFromMem = 0;

long endWashLastToggleTime = 0;
long endWashToggleTime = 0;
long endWashToggleTimeFromMem = 0;

long uptime = 0;

int redPin = 6;
int greenPin = 9;
int bluePin = 10;

int LEDstate = HIGH;      // the current state of the output pin

long tiltSensor1Baseline = 0;
int tiltSensorReading;           // the current tiltSensorReading from the input pin
int tiltSensorStatus = 0;
long tiltSensorLastToggleTime = 0;         // the last tiltSensorLastToggleTime the output pin was toggled
long tiltSensorToggleTime = 0;
long tiltSensorToggleTimeFromMem = 0;
long debounce = 50;   // the debounce tiltSensorLastToggleTime, increase if the output flickers

long millisLast = 0;

int dishwasherState = 0;

long lastConnectionTime = 0; 

const int updateInterval = 30 * 1000;

void setup(void)
{
  digitalWrite(resetPin, HIGH);
  delay(200);
  pinMode(resetPin, OUTPUT);
  
  setColor(150, 0, 0);
  
  Serial.begin(115200);
  Serial.println(F("Starting TinkurWash..."));

  tiltSensorStatus = 0;
  
  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  
  Serial.println(F("[SRAM Check]"));
  Serial.println(freeRam());
  
  //RGB LED
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  /* Initialise the module */
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
  
  // Optional SSID scan
  // listSSIDResults();
  
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100); 
    
    /*
    //DHCP Timeout after 10 tries
    dhcpTimeoutTimer += 1;
    
    if(dhcpTimeoutTimer >= 10) {
      resetTinkurWash();
    }
    */
  }  

  ip = 0;
  // Try looking up the website's IP address
  Serial.print(WEBSITE); Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }

  cc3000.printIPdotsRev(ip);
  
  Serial.println();
  Serial.println();
  
  //knockSensor1Baseline = 100; //for testing
  knockSensor1Baseline = vibrationReading(); //for production
  Serial.print(knockSensor1Baseline);
  Serial.println(F(" baseline knocks"));
  
  tiltSensor1Baseline = readAxis(zInput);
  Serial.print(tiltSensor1Baseline);
  Serial.println(F(" baseline tilt"));
  
  Serial.println();
  Serial.println();
  
  //read existing settings struct values fro EEPROM Memory 
  EEPROM_readAnything(0, configuration);
  
  Serial.println("");
  Serial.print("configuration.selfrestart = ");
  Serial.println(configuration.selfrestart);
  Serial.println("");
  
  if(configuration.selfrestart == 1) {
     setColor(150, 150, 150); //white
     
    //immediatly reset self restart to false
    configuration.selfrestart = 0;
    EEPROM_writeAnything(0, configuration);
    
    //update variables to previous TinkurWash states
    tiltSensorStatus = configuration.tiltSensorStatus;
    dishwasherState = configuration.dishwasherState;
    knockSensor1Baseline = configuration.knockSensor1Baseline;
    tiltSensor1Baseline = configuration.tiltSensor1Baseline;
    knockSensorToggleTimeFromMem = configuration.knockSensorToggleTime;
    tiltSensorToggleTimeFromMem = configuration.tiltSensorToggleTime;
    endWashToggleTimeFromMem = configuration.endWashToggleTime;
    
    Serial.print("tiltSensorStatus: ");
    Serial.println(tiltSensorStatus);
    
    Serial.print("dishwasherState: ");
    Serial.println(dishwasherState);
    
    Serial.print("knockSensor1Baseline: ");
    Serial.println(knockSensor1Baseline);
    
    Serial.print("tiltSensor1Baseline: ");
    Serial.println(tiltSensor1Baseline);
    
    Serial.print("knockSensorToggleTime: ");
    Serial.println(knockSensorToggleTimeFromMem);
    
    Serial.print("tiltSensorToggleTime: ");
    Serial.println(tiltSensorToggleTimeFromMem);
    
    Serial.print("endWashToggleTime: ");
    Serial.println(endWashToggleTimeFromMem);
    
  } else {
    
    setColor(150, 0, 0); //red
  }
  
}

void loop(void)
{
  
  watchdogEnable();
  
  tiltSensorStatus = tiltReading();
  
  //don't record knocks if the dishwasher is open
  if((tiltSensorStatus == 0) && (((tiltSensorToggleTime) * .001) > 5))
  {
    
    //Knock Instrumentation
    knockSensor1Interval = vibrationReading();
    
    knockSensor1Last10.add(knockSensor1Interval);
    
    knockSensor1Median = knockSensor1Last10.getMedian();
    
    loggingKnocks = true;
  } else {
    loggingKnocks = false;
    delay(1000);
  }
  
  if(knockSensor1Last10.getCount() == 10)
  {
  
     //if (knockSensor1Median > (knockSensor1Baseline * 3) && (dishwasherState == 0))
     if (knockSensor1Median > (knockSensor1Baseline + 100) && (dishwasherState == 0))
     {
        dishwasherState = 1;
     }
     
     //if (knockSensor1Median > (knockSensor1Baseline * 3))
     if (knockSensor1Median > (knockSensor1Baseline + 100))
     {   
       knockSensorToggleTimeFromMem = 0; 
       
       knockSensorLastToggleTime = millis();
     }
     
     knockSensorToggleTime = (knockSensorToggleTimeFromMem) + (millis() - knockSensorLastToggleTime);
     
     if ((((knockSensorToggleTime) * .001) > (washCompleteSeconds)) && (dishwasherState == 1))
     {
        dishwasherState = 2;
        
        endWashToggleTimeFromMem = 0;
        
        endWashLastToggleTime = millis();
        
     }
     
     if (dishwasherState == 2)
      {
        endWashToggleTime = (endWashToggleTimeFromMem) + (millis() - endWashLastToggleTime);
      }
     
     if ((((endWashToggleTime) * .001) > (dryCompleteSeconds)) && (dishwasherState == 2))
     {
        dishwasherState = 3;
     }
     
     if (((tiltSensorToggleTime * .001) > doorOpenCleanSeconds) && (tiltSensorStatus == 1) && ((dishwasherState == 2) || (dishwasherState == 3)))
     {
        dishwasherState = 0;
     }
     
     
     //Set REGB LED and determine dishwasher status
     if (dishwasherState == 0)
     {
        setColor(0, 150, 0);  // green
     }
     
     if (dishwasherState == 1)
     {
       int i = 0;
      
      while(i < 1){
        int j = 0;
        
        while (j < 1) {
           setColor(0, 0, 5);  // blue
           delay(200);
           setColor(0, 0, 150);  // blue
           delay(100);
         
           j += 1;
        } 
        
        i += 1;
        
      } 
       
     }
     
     if (dishwasherState == 2)
     {
        setColor(0, 0, 150);  // blue
     }
     
     if (dishwasherState == 3)
     {
        setColor(150, 150, 0);  // yellow
     }
      
    // convert the data to a String
    String dataString = "knocks,";
    dataString += String(knockSensor1Median);
    
    // you can append multiple readings to this String to 
    // send the xively feed multiple values
    dataString += "\ntilt,";
    dataString += String(tiltSensorStatus);
    
    
    // you can append multiple readings to this String to 
    // send the xively feed multiple values
    dataString += "\nstatus,";
    dataString += String(dishwasherState);
    
    // you can append multiple readings to this String to 
    // send the xively feed multiple values
    dataString += "\nuptime,";
    dataString += String(uptime);
    
    // you can append multiple readings to this String to 
    // send the xively feed multiple values
    dataString += "\ntiltvalue,";
    dataString += String(readAxis(zInput));  
    
    if((millis() - lastConnectionTime > updateInterval))
    {
    
      sendData(dataString);
    
    }
    
    delay(100);   
  }
  
  uptime = (millis() * .001);
  
  if(verboseLoggingEnabled) {
      Serial.print("Logging Knocks: ");
      Serial.println(loggingKnocks);
      
      Serial.print(F("Last High Knock seconds ago: "));
      Serial.println((knockSensorToggleTime) * .001);
      
      Serial.print(readAxis(xInput));
      Serial.print(" / ");
      Serial.print(readAxis(yInput));
      Serial.print(" / ");
      Serial.println(readAxis(zInput));
      
      Serial.print(knockSensor1Median);
      Serial.println(F(" ten interval knock median"));
      
      Serial.print(knockSensor1Interval);
      Serial.println(F(" knocks last interval"));
      
      Serial.print(tiltSensorStatus);
      Serial.print(F(" tilt for "));
      Serial.print((millis() - tiltSensorLastToggleTime) * .001);
      Serial.println(F(" seconds"));
      
      Serial.print(F("Median Count: "));
      Serial.println(knockSensor1Last10.getCount());
      
      Serial.print(F("Uptime: "));
      Serial.print(uptime);
      Serial.println(F(" seconds"));
      
      if (dishwasherState == 2)
      {
        Serial.print(F("End Wash Toggle Time seconds ago: "));
        Serial.println((endWashToggleTime) * .001); 
      }
      
      if (dishwasherState == 0)
      {
        Serial.println(F("Dishwasher OFF"));
      }
     
      if (dishwasherState == 1)
      {
        Serial.println(F("Dishwasher ON - Wash Cycle"));
      }
     
      if (dishwasherState == 2)
      {
        Serial.println(F("Dishwasher ON - Dry Cycle"));
      }
     
      if (dishwasherState == 3)
      {
        Serial.println(F("Dishwasher Clean"));
      }
    
      Serial.println();
  }
  
  /*
  //to test reset and restore from memory
  if (millis() >= 100000) {
    resetTinkurWash();
  }
  */
  
  wdt_disable();
}

bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void sendData(String thisData) {

  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, 80);
    if (www.connected()) {
      
       Serial.println(F("connecting..."));
      // send the HTTP PUT request:
      www.print(F("PUT /v2/feeds/"));
      www.print(FEEDID);
      www.println(F(".csv HTTP/1.0"));
      www.println(F("Host: api.xively.com"));
      www.print(F("X-ApiKey: "));
      www.println(APIKEY);
      www.print(F("Content-Length: "));
      www.println(thisData.length());
  
      // last pieces of the HTTP PUT request:
      www.println(F("Content-Type: text/csv"));
      www.println(F("Connection: close"));
      www.println();
  
      // here's the actual content of the PUT request:
      www.println(thisData);
      Serial.println(thisData);
      
      www.println();
      
      lastConnectionTime = millis();
      
      Serial.println();
      Serial.println(F("[SRAM Check]"));
      Serial.println(freeRam());
      Serial.println();
      
      //reset number of failed network connections
      networkConnectionFailTimer = 0;
      
    } else {
      Serial.println(F("Connection failed"));   
    
      //failed network connection reset when 3 failures
      networkConnectionFailTimer += 1;
    
      if(networkConnectionFailTimer >= 3) {
      resetTinkurWash();
      } 
      
      return;
    }
  
    Serial.println(F("-------------------------------------"));
    
    /* Read data until either the connection is closed, or the idle timeout is reached. */ 
    unsigned long lastRead = millis();
    while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {
      while (www.available()) {
        char c = www.read();
        Serial.print(c);
        lastRead = millis();
      }
    }
    
    
    www.close();
    Serial.println(F("-------------------------------------"));
    
    /*
    // You need to make sure to clean up after yourself or the CC3000 can freak out 
    // the next time your try to connect ... 
    Serial.println(F("\n\nDisconnecting"));
    cc3000.disconnect();
    */
    
  
  }

int tiltReading()
{
  
    int previousTiltSensorStatus = tiltSensorStatus;
    
    if (readAxis(zInput) < (tiltSensor1Baseline - 30)) {
      tiltSensorStatus = 1;
    }
    else {
      tiltSensorStatus = 0;
    }
      
    tiltSensorToggleTime = (tiltSensorToggleTimeFromMem) + (millis() - tiltSensorLastToggleTime);
    
    if (tiltSensorStatus != previousTiltSensorStatus) {
      
      tiltSensorToggleTimeFromMem = 0;
      
      tiltSensorLastToggleTime = millis();
    } 
      
  return tiltSensorStatus;
}

void setColor(int red, int green, int blue)
{
  analogWrite(redPin, 255-red);
  analogWrite(greenPin, 255-green);
  analogWrite(bluePin, 255-blue);
}

int readAxis(int axisPin)
{
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
  return reading/sampleSize;
}

int changeInAxis(int axisPin)
{
  long reading1 = 0;
  long reading2 = 0;  
  
  reading1 = analogRead(axisPin);
  delay(1);
  reading2 = analogRead(axisPin);
 
  return abs(reading1 - reading2);
}

long vibrationReading()
{
  millisLast = millis();
  
  long totalX = 0;
  long totalY = 0;
  long totalZ = 0;
  
  while (millis() - millisLast < (intervalSeconds * 1000))
  {
    totalX = totalX + (changeInAxis(xInput));
    totalY = totalY + (changeInAxis(yInput));
    totalZ = totalZ + (changeInAxis(zInput));
  }
  
  return totalX + totalY + totalZ;
}

void watchdogEnable()
{
  counter=0;
  cli();                              // disable interrupts

  MCUSR = 0;                          // reset status register flags

                                      // Put timer in interrupt-only mode:                                        
  WDTCSR |= 0b00011000;               // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                      // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001;    // set WDIE (interrupt enable...7th from left, on left side of bar)
                                      // clr WDE (reset enable...4th from left)
                                      // and set delay interval (right side of bar) to 8 seconds,
                                      // using bitwise OR operator.

  sei();                              // re-enable interrupts
  

  // delay interval patterns:
  //  16 ms:     0b000000
  //  500 ms:    0b000101
  //  1 second:  0b000110
  //  2 seconds: 0b000111
  //  4 seconds: 0b100000
  //  8 seconds: 0b100001

}

ISR(WDT_vect) // watchdog timer interrupt service routine
{
  counter+=1;

  if (counter < countmax)
  {
    wdt_reset(); // start timer again (in interrupt-only mode)
  }
  else             // then change timer to reset-only mode with short (16 ms) fuse
  {
    resetTinkurWash();
  }
}

void resetTinkurWash()
{
    Serial.println("Resetting TinkurWash!!!");
    
    //write settings struct values fro EEPROM Memory  
    configuration.selfrestart = 1;
    configuration.dishwasherState = dishwasherState;
    configuration.tiltSensorStatus = tiltSensorStatus;
    configuration.knockSensor1Baseline = knockSensor1Baseline;
    configuration.tiltSensor1Baseline = tiltSensor1Baseline;
    configuration.knockSensorToggleTime = knockSensorToggleTime;
    configuration.tiltSensorToggleTime = tiltSensorToggleTime;
    configuration.endWashToggleTime = endWashToggleTime;
    
    EEPROM_writeAnything(0, configuration);
    
    delay(5000);
    
    digitalWrite(resetPin, LOW);
                                        
    delay(5000);
}
