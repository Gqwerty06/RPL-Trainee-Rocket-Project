#include <Arduino.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <SD.h>
#include <stdarg.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include <FS.h>


void setup(){
    Serial.begin(115200);
    Serial.println("please show");

}

void loop(){
    Serial.println(Serial.read());
    int x = 10;
    while(x<10){
        Serial.println(x);
        x++;
    }
    Serial.println("program finish hope no fries");
}