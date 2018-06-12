#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*
 * Actuator PIN declaration.
 * VERBOSE MODE FOR DEBUG OUTPUT.
*/
#define RED_LED                    5
#define GREEN_LED                  12
#define BUZZER                     A0
#define VERBOSE_MODE               true

/*
 * Variables declaration.
*/
String item = "";
bool check = 1;

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  //Action to be performed should be coded here//
  while (1);
}

void setup() {
//  while (!Serial); // required for Flora & Micro
//  delay(500);
  
  Serial.begin(115200);
  Serial.println(F("BIBO: Be In Be Out"));
  Serial.println(F("---------------------------------------------------"));

  randomSeed(micros());

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  /*commenting out the factory reset because it erases the custom GATT services that we have made
    from the non-volatile memmory*/
//  Serial.println(F("Performing a factory reset: "));
//  if (! ble.factoryReset() ){
//       error(F("Couldn't factory reset"));
//  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  // this line is particularly required for Flora, but is a good idea
  // anyways for the super long lines ahead!
  // ble.setInterCharWriteDelay(5); // 5 ms

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'BIBO 1.0': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=BIBO 1.0")) ) {
    error(F("Could not set device name"));
  } 
  /*Set advertising interval to the minimum: 20ms*/
  ble.println("AT+GAPINTERVALS=,,20,");
  if(!ble.waitForOK()){
    error(F("Could not set advertising interval"));
  }
  
  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();
}

void loop() {
  
  ble.println("AT+GATTCHAR=1,BIBO"); 
  if ( !ble.waitForOK() )
  {
    error(F("Failed to get response from notify property update"));
  }
  
  item = "";
  check = 1;

  while(ble.isConnected()){
    if(check == 1){
      check = 0;
        
      digitalWrite(GREEN_LED,HIGH);
      digitalWrite(RED_LED,HIGH);
      analogWrite(BUZZER,255);
      delay(100);
      digitalWrite(GREEN_LED,LOW);  
      analogWrite(BUZZER,0);
      digitalWrite(RED_LED,LOW);
    }

    ble.println(F("AT+BLEUARTTX=IN:7X"));
    ble.waitForOK();

    ble.println(F("+++\r\n"));
    if ( !ble.waitForOK() )
    {
      error(F("Mode change Failed"));
    }

    item = "";
    while(ble.available()){
        int c = ble.read();
        item += char(c);
    }  
    
    if(item != ""){
      Serial.println("******************************************************Got: " + item);
      if(item == "1"){
        digitalWrite(GREEN_LED,HIGH);
        delay(200);
        digitalWrite(GREEN_LED,LOW);
        delay(100);
        digitalWrite(GREEN_LED,HIGH);
        delay(200);
        digitalWrite(GREEN_LED,LOW);
      }
      if(item == "0"){
        digitalWrite(RED_LED,HIGH);
        analogWrite(BUZZER,255);
        delay(400);
        analogWrite(BUZZER,0);
        digitalWrite(RED_LED,LOW);
      }
    }

    ble.println(F("+++\r\n"));
    if ( !ble.waitForOK() )
    {
      error(F("Mode change Failed"));
    }

    delay(20);
  }
  if(check == 0){
    check = 1;
  }
  delay(20);
}
