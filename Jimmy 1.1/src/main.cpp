//nrf RX Code
#include <Arduino.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <stdarg.h>
#include <Wire.h>

RF24 radio; // CE, CSN

//declare gyrovars


const byte addy[][6] = {"00001","00002"};
const uint8_t chan = 105;

struct data
{
  int16_t lat = 0, lon = 1, alt = 0, ax = 2, ay = 3, az = 4, gx = 0, gy = 1, gz= 0;
};
data dat1, dat2;

void setup()
{
  radio.begin(21,5);
  radio.setPALevel(RF24_PA_MAX, 0);
  radio.setChannel(chan);
  Serial.begin(115200);
  Serial.println("Starting to read: ");
  radio.openReadingPipe(1, addy[0]);

  //IMU int.
  //Wire.begin(21,22);
  //mpu.initialize();
}

void recData();

String buff, buf;
void loop()
{
  //readIMU();
  recData();
  delay(75);
}

void recData()
{
   radio.startListening();
   if(radio.available())
   {
     radio.read(&dat1, sizeof(data));
     buf = String(dat1.lat) + "\t" + String(dat1.lon) + "\t" + String(dat1.alt) + "\t" + String(dat1.ax) + "\t" + String(dat1.ay)
    + "\t" + String(dat1.az) + "\t" + String(dat1.gx) + "\t" + String(dat1.gy) + "\t" + String(dat1.gz);
     Serial.println("Receiving: ");
     Serial.println(buf);
   }
}