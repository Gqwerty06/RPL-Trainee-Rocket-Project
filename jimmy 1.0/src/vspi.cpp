//nrf TX Code using VSPI only because I can't figure out dual
//Code by Gabriel Qu
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

//pin declares for chip selects
#define RF_CS 5
#define SD_CS 2

//radio setups
const byte addy[][6] = {"00001","00002"};
const uint8_t chan = 105;

//reference altitude float
float refAlt;

//Module declares
SoftwareSerial gpsSerial(17, 16); //gps is UART so different? TBH I'm not that sure
TinyGPSPlus gps;
RF24 radio; // CE, CSN
Adafruit_BMP3XX bmp;
MPU6050 mpu; // Set up IMU
File file;

SoftwareSerial ss(17, 16); // for the gps

struct data
{
  int16_t ax, ay, az, gx, gy, gz, satNum;
  float lat, lon, alt;
};
data dat1;

String buff, buf;

//method declares for code organization - if you write your methods above where they are declared
// there is no need for this. 

void sendData();
void SDWrite();
void dataGather();

void setup()
{
  //radio initializations
  radio.begin(4,5);
  radio.setPALevel(RF24_PA_MAX, 0);
  radio.setChannel(chan);
  Serial.begin(115200);
  Serial.println("Starting to send: ");
  radio.openWritingPipe(addy[0]);

  //imu mpu and gps start declare
  bmp.begin_I2C();
  mpu.initialize();
  gpsSerial.begin(9600); // connect gps sensor, default baud rate for L80 is 9600

  //BMP390 setup- oversampling and filtering
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  //refernce altitude for calculation
  refAlt = bmp.readAltitude(1013.25);
}

void loop()
{
  delay(50);
  dataGather();
  sendData();
  file = SD.open("file", FILE_WRITE);
  if (file){
    SDWrite();
  }
}

/* 
Simple Code explanation for this method (I'm no expert so google it more):
Since the CS is active low, which is to say the chip only works when the pin is not powered,
and you want to use two SPI Peripherals on a single SPI bus, you need to tell a peripheral 
when to turn on and off in order for data to transmit only to that peripheral. By doing digitalWrite
before and after the data transmission, you are effectively turning the module on for a short while to
send data, and then off so it doesn't interfere with the SD card write method. 

For more in depth information on the radio. methods, check the RF library this code utilizes.
*/
void sendData()
{
  digitalWrite(RF_CS, LOW);
  radio.stopListening();
  buff = String(dat1.lat, 6) + "\t" + String(dat1.lon, 6) + "\t" + String(dat1.alt) + "\t" + String(dat1.satNum) + "\t" + String(dat1.ax) + "\t" + String(dat1.ay)
    + "\t" + String(dat1.az) + "\t" + String(dat1.gx) + "\t" + String(dat1.gy) + "\t" + String(dat1.gz);
  Serial.println("Sending: ");
  Serial.println(buff);
  radio.write(&dat1, sizeof(data));
  digitalWrite(RF_CS, HIGH);
}

/*
Unfinished, but the default SD.h method by arduino has functionality for this
*/
void SDWrite()
{
  digitalWrite(SD_CS, LOW);
  file.println(buff);
  digitalWrite(SD_CS, HIGH);
}

/*
Check your module compatability and whatnot for this section of code, 
we use the BMP390, the jimmy IMU (BMP085?)and the Quectal L80 GPS
*/
void dataGather()
{
  //lat long data get- this code uses the tinyGPSPLus Lib, check parts for compatabilty 
   while (gpsSerial.available())     // check for gps data
  {
    if (gps.encode(gpsSerial.read()))   // encode gps data
    {
      dat1.lat= gps.location.lat();
      dat1.lon= gps.location.lng();
    }
  }
  //mpu data write
  mpu.getMotion6(&dat1.ax, &dat1.ay, &dat1.az, &dat1.gx, &dat1.gy, &dat1.gz);
  
  //bmp altitude get
  dat1.alt = (bmp.readAltitude(1013.25)); //average value, measure and change onsite for more accurate measurement
}

//launch states- change data collection rate and operations
//Zero sensors at launch site
//physical seperation between recovery system and power9