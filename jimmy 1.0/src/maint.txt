//nrf TX Code
#include <Arduino.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <SD.h>
#include <stdarg.h>


//Uninitialized pointers to null SPI Items
SPIClass *vspi = NULL;
SPIClass *hspi = NULL;


RF24 radio; // CE, CSN

const byte addy[][6] = {"00001","00002"};
const uint8_t chan = 105;


struct data
{
  int16_t ax = 0, ay = 1, az = 2, gx = 3, gy = 4, gz = 5;
};
data dat1;

void setup()
{
  radio.begin(27,15);
  radio.setPALevel(RF24_PA_MAX, 0);
  radio.setChannel(chan);
  Serial.begin(115200);
  Serial.println("Starting to send: ");
  radio.openWritingPipe(addy[0]);

  //SPI Declarations
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  vspi->begin();
  hspi->begin();

  //set up slave select pins as outputs, Arduino API doesn't handle automatically pulling SS low
  pinMode(vspi->pinSS(), OUTPUT);  //VSPI SS
  pinMode(hspi->pinSS(), OUTPUT);  //HSPI SS

}

String buff, buf;
void loop()
{
  sendData();
  delay(75);
}


void sendData()
{
  radio.stopListening();
  buff = String(dat1.ax) + "\t" + String(dat1.ay) + "\t" + String(dat1.az) + "\t" + String(dat1.gx) + "\t" + String(dat1.gy)
    + "\t" + String(dat1.gz);
  Serial.println("Sending: ");
  Serial.println(buff);
  radio.write(&dat1, sizeof(data));
}

